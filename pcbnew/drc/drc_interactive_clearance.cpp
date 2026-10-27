/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <drc/drc_interactive_clearance.h>

#include <algorithm>

#include <board_design_settings.h>
#include <core/kicad_algo.h>
#include <drc/drc_engine.h>
#include <drc/drc_rule.h>
#include <footprint.h>
#include <gal/color4d.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_segment.h>
#include <pad.h>
#include <pcb_shape.h>
#include <pcb_track.h>
#include <view/view.h>
#include <view/view_overlay.h>


// The copper-bearing item types, as indexed by DRC_CACHE_GENERATOR less the rarely-copper ones
static const std::vector<KICAD_T> s_copperTypes = { PCB_TRACE_T, PCB_ARC_T,  PCB_VIA_T,
                                                    PCB_PAD_T,   PCB_SHAPE_T, PCB_FIELD_T,
                                                    PCB_TEXT_T,  PCB_TEXTBOX_T };


/// The copper layers an item is tested on.  A pad hole pierces all of them.
static LSET copperLayersOf( const BOARD_ITEM* aItem, const LSET& aBoardCopper )
{
    if( aItem->Type() == PCB_PAD_T && aItem->HasHole() )
        return aBoardCopper;

    return aItem->GetLayerSet() & aBoardCopper;
}


/// What DRC tests on a copper layer: only the hole where a pad or via is not flashed.
static std::shared_ptr<SHAPE> copperShapeOf( const BOARD_ITEM* aItem, PCB_LAYER_ID aLayer )
{
    if( aItem->Type() == PCB_PAD_T && !static_cast<const PAD*>( aItem )->FlashLayer( aLayer ) )
    {
        if( aItem->HasHole() )
            return aItem->GetEffectiveHoleShape();

        return nullptr;
    }

    if( aItem->Type() == PCB_VIA_T && !static_cast<const PCB_VIA*>( aItem )->FlashLayer( aLayer ) )
        return aItem->GetEffectiveHoleShape();

    return aItem->GetEffectiveShape( aLayer );
}


/// An unflashed NPTH has no copper, so on that layer it only has hole clearances.
static bool isBareHole( const BOARD_ITEM* aItem, PCB_LAYER_ID aLayer )
{
    if( aItem->Type() != PCB_PAD_T )
        return false;

    const PAD* pad = static_cast<const PAD*>( aItem );

    return pad->GetAttribute() == PAD_ATTRIB::NPTH && !pad->FlashLayer( aLayer );
}


/// DRC only counts a via hole on the layers the via spans.
static bool hasHoleOn( const BOARD_ITEM* aItem, PCB_LAYER_ID aLayer )
{
    if( aItem->Type() == PCB_VIA_T )
        return aItem->GetLayerSet().Contains( aLayer );

    return aItem->HasHole();
}


/// Net code for the same-net waivers; -1 for items that cannot carry a net.
static int netCodeOf( const BOARD_ITEM* aItem )
{
    if( aItem->IsConnected() )
        return static_cast<const BOARD_CONNECTED_ITEM*>( aItem )->GetNetCode();

    return -1;
}


DRC_INTERACTIVE_CLEARANCE::DRC_INTERACTIVE_CLEARANCE( const std::shared_ptr<DRC_ENGINE>& aDRCEngine ) :
        DRC_TEST_PROVIDER(),
        m_largestClearance( 0 ),
        m_largestEdgeClearance( 0 ),
        m_epsilon( 0 )
{
    SetDRCEngine( aDRCEngine.get() );
}


DRC_INTERACTIVE_CLEARANCE::~DRC_INTERACTIVE_CLEARANCE() = default;


void DRC_INTERACTIVE_CLEARANCE::Init( BOARD* aBoard, const std::vector<BOARD_ITEM*>& aMovingItems )
{
    m_board = aBoard;
    m_movingItems.clear();
    m_edges.clear();
    m_copperTree.clear();
    m_edgeTree.clear();
    m_clearanceCache.clear();
    m_violations.clear();

    m_epsilon = m_board->GetDesignSettings().GetDRCEpsilon();

    // Already includes the worst clearance and hole clearance rules
    m_largestClearance = m_board->GetMaxClearanceValue();

    DRC_CONSTRAINT worst;
    m_largestEdgeClearance = m_drcEngine->QueryWorstConstraint( EDGE_CLEARANCE_CONSTRAINT, worst )
                                     ? worst.GetValue().Min() : 0;

    std::set<BOARD_ITEM*> moving;

    auto addMoving =
            [&]( BOARD_ITEM* aItem )
            {
                // The Move tool lists a footprint's pads as well as the footprint
                if( !moving.insert( aItem ).second )
                    return;

                if( alg::contains( s_copperTypes, aItem->Type() ) && !isInvisibleText( aItem )
                        && ( aItem->IsOnCopperLayer() || aItem->HasHole() ) )
                {
                    m_movingItems.push_back( aItem );
                }
            };

    for( BOARD_ITEM* item : aMovingItems )
    {
        addMoving( item );
        item->RunOnChildren( addMoving, RECURSE_MODE::RECURSE );
    }

    if( m_movingItems.empty() )
        return;

    // The whole board is indexed, which is a noticeable pause only on very large boards
    LSET boardCopper = LSET::AllCuMask( m_board->GetCopperLayerCount() );

    forEachGeometryItem( s_copperTypes, boardCopper,
            [&]( BOARD_ITEM* item ) -> bool
            {
                if( !moving.count( item ) )
                {
                    for( PCB_LAYER_ID layer : copperLayersOf( item, boardCopper ) )
                        m_copperTree.Insert( item, layer, m_largestClearance );
                }

                return true;
            } );

    forEachGeometryItem( { PCB_SHAPE_T }, LSET( { Edge_Cuts, Margin } ),
            [&]( BOARD_ITEM* item ) -> bool
            {
                if( !moving.count( item ) )
                {
                    // DRC measures to the middle of an edge cut line, so with no width
                    PCB_SHAPE* edge = static_cast<PCB_SHAPE*>( item->Clone() );

                    if( edge->IsOnLayer( Edge_Cuts ) )
                        edge->SetWidth( 0 );

                    m_edges.emplace_back( edge );

                    for( PCB_LAYER_ID layer : { Edge_Cuts, Margin } )
                    {
                        if( item->IsOnLayer( layer ) )
                            m_edgeTree.Insert( m_edges.back().get(), layer, m_largestEdgeClearance );
                    }
                }

                return true;
            } );

    // DRC treats a non-round NPTH as milled, so it takes edge clearances like the board outline
    forEachGeometryItem( { PCB_PAD_T }, LSET::AllLayersMask(),
            [&]( BOARD_ITEM* item ) -> bool
            {
                PAD* pad = static_cast<PAD*>( item );

                if( !moving.count( pad ) && pad->GetAttribute() == PAD_ATTRIB::NPTH && pad->HasHole()
                        && pad->GetDrillSizeX() != pad->GetDrillSizeY() )
                {
                    m_edgeTree.Insert( pad, Edge_Cuts, m_largestEdgeClearance );
                }

                return true;
            } );
}


bool DRC_INTERACTIVE_CLEARANCE::Run()
{
    if( m_movingItems.empty() )
        return true;

    LSET boardCopper = LSET::AllCuMask( m_board->GetCopperLayerCount() );

    m_violations.clear();
    m_reported.clear();

    for( BOARD_ITEM* item : m_movingItems )
    {
        for( PCB_LAYER_ID layer : copperLayersOf( item, boardCopper ) )
        {
            std::shared_ptr<SHAPE> shape = copperShapeOf( item, layer );

            if( !shape )
                continue;

            // Tree entries are inflated by the largest clearance, so the bare box finds them all
            std::set<BOARD_ITEM*> others;

            for( DRC_RTREE::ITEM_WITH_SHAPE* entry : m_copperTree.Overlapping( layer, shape->BBox() ) )
                others.insert( entry->parent );

            for( BOARD_ITEM* other : others )
                testCopper( item, shape, layer, other );

            // Castellated pads and edge connector fingers sit on the board edge by design
            PAD* pad = dynamic_cast<PAD*>( item );

            if( pad && ( pad->GetProperty() == PAD_PROP::CASTELLATED || pad->GetAttribute() == PAD_ATTRIB::CONN ) )
                continue;

            std::set<BOARD_ITEM*> edges;

            for( PCB_LAYER_ID edgeLayer : { Edge_Cuts, Margin } )
            {
                for( DRC_RTREE::ITEM_WITH_SHAPE* entry : m_edgeTree.Overlapping( edgeLayer, shape->BBox() ) )
                    edges.insert( entry->parent );
            }

            for( BOARD_ITEM* edge : edges )
            {
                std::shared_ptr<SHAPE> edgeShape = edge->GetEffectiveShape( Edge_Cuts );
                int clearance = getClearance( EDGE_CLEARANCE_CONSTRAINT, edge, item, UNDEFINED_LAYER );

                if( collide( shape.get(), edgeShape.get(), clearance ) )
                    report( item, edge, edgeShape, clearance );
            }
        }
    }

    return true;
}


int DRC_INTERACTIVE_CLEARANCE::getClearance( DRC_CONSTRAINT_T aType, const BOARD_ITEM* a,
                                             const BOARD_ITEM* b, PCB_LAYER_ID aLayer )
{
    auto key = std::make_tuple( aType, a, b, aLayer );
    auto it = m_clearanceCache.find( key );

    if( it == m_clearanceCache.end() )
    {
        DRC_CONSTRAINT constraint = m_drcEngine->EvalRules( aType, a, b, aLayer );
        int            clearance = -1;

        if( constraint.GetSeverity() != RPT_SEVERITY_IGNORE )
            clearance = constraint.GetValue().Min();

        it = m_clearanceCache.emplace( key, clearance ).first;
    }

    return it->second;
}


bool DRC_INTERACTIVE_CLEARANCE::collide( const SHAPE* aShape, const SHAPE* aOther, int aClearance ) const
{
    // Like DRC, allow for the rounding of values that sit exactly on the limit
    return aClearance >= 0 && aShape->Collide( aOther, std::max( 0, aClearance - m_epsilon ) );
}


void DRC_INTERACTIVE_CLEARANCE::testCopper( BOARD_ITEM* aItem, const std::shared_ptr<SHAPE>& aShape,
                                            PCB_LAYER_ID aLayer, BOARD_ITEM* aOther )
{
    int  netCode = netCodeOf( aItem );
    bool bothPads = aItem->Type() == PCB_PAD_T && aOther->Type() == PCB_PAD_T;

    // Copper of one net may touch, holes and all.  As in DRC, pads only waive a real net, while
    // other unconnected copper doesn't clash with unconnected copper either.
    if( netCode == netCodeOf( aOther ) && ( netCode > 0 || !bothPads ) )
        return;

    // Pads of one footprint sharing a number are one pad
    if( bothPads && static_cast<PAD*>( aItem )->SameLogicalPadAs( static_cast<PAD*>( aOther ) ) )
        return;

    std::shared_ptr<SHAPE> otherShape = copperShapeOf( aOther, aLayer );

    if( !otherShape )
        return;

    if( !isBareHole( aItem, aLayer ) && !isBareHole( aOther, aLayer ) )
    {
        int clearance = getClearance( CLEARANCE_CONSTRAINT, aItem, aOther, aLayer );

        if( clearance > 0 && collide( aShape.get(), otherShape.get(), clearance ) )
        {
            report( aItem, aOther, otherShape, clearance );
            return;
        }
    }

    // A hole may not even touch foreign copper, so a zero hole clearance is still tested
    if( hasHoleOn( aItem, aLayer ) )
    {
        std::shared_ptr<SHAPE_SEGMENT> hole = aItem->GetEffectiveHoleShape();
        int clearance = getClearance( HOLE_CLEARANCE_CONSTRAINT, aItem, aOther, aLayer );

        if( collide( hole.get(), otherShape.get(), clearance ) )
        {
            report( aItem, aOther, otherShape, clearance );
            return;
        }
    }

    if( hasHoleOn( aOther, aLayer ) )
    {
        std::shared_ptr<SHAPE_SEGMENT> otherHole = aOther->GetEffectiveHoleShape();
        int clearance = getClearance( HOLE_CLEARANCE_CONSTRAINT, aOther, aItem, aLayer );

        if( collide( aShape.get(), otherHole.get(), clearance ) )
            report( aItem, aOther, otherHole, clearance );
    }
}


void DRC_INTERACTIVE_CLEARANCE::report( BOARD_ITEM* aItem, BOARD_ITEM* aOther,
                                        std::shared_ptr<SHAPE> aOtherShape, int aClearance )
{
    // A pair may collide on several layers; one mark is enough
    if( m_reported.insert( { aItem, aOther } ).second )
        m_violations.push_back( { aOther, std::move( aOtherShape ), aClearance } );
}


void DRC_INTERACTIVE_CLEARANCE::UpdateConflicts( KIGFX::VIEW* aView )
{
    if( !m_overlay )
    {
        if( m_violations.empty() )
            return;

        m_overlay = aView->MakeOverlay();
    }

    int                         maxError = m_board->GetDesignSettings().m_MaxError;
    std::vector<SHAPE_POLY_SET> items;

    m_overlay->Clear();
    m_overlay->SetIsFill( true );
    m_overlay->SetIsStroke( false );

    // As the router shows a collision: the item in green over the keep-out its clearance asks
    // for.  All keep-outs go first, so none covers another item.
    m_overlay->SetFillColor( COLOR4D( DARKDARKGRAY ).WithAlpha( 0.7 ) );

    for( const VIOLATION& violation : m_violations )
    {
        SHAPE_POLY_SET& item = items.emplace_back();
        SHAPE_POLY_SET  keepOut;

        violation.m_shape->TransformToPolygon( item, maxError, ERROR_OUTSIDE );

        // An edge cut is measured to its middle, a line with no area to inflate
        if( violation.m_item->IsOnLayer( Edge_Cuts ) || violation.m_item->IsOnLayer( Margin ) )
        {
            violation.m_item->TransformShapeToPolygon( keepOut, Edge_Cuts, violation.m_clearance, maxError,
                                                       ERROR_OUTSIDE );
        }
        else
        {
            keepOut = item;
            keepOut.Inflate( violation.m_clearance, CORNER_STRATEGY::ROUND_ALL_CORNERS, maxError );
        }

        m_overlay->Polygon( keepOut );
    }

    m_overlay->SetFillColor( COLOR4D( 0, 1, 0, 1 ) );

    for( const SHAPE_POLY_SET& item : items )
        m_overlay->Polygon( item );

    aView->Update( m_overlay.get() );
}


void DRC_INTERACTIVE_CLEARANCE::ClearConflicts( KIGFX::VIEW* aView )
{
    if( m_overlay )
    {
        m_overlay->Clear();
        aView->Remove( m_overlay.get() );
        m_overlay.reset();
    }
}
