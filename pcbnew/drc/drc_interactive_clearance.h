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

#pragma once

#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include <drc/drc_rtree.h>
#include <drc/drc_test_provider.h>

class PCB_SHAPE;
class SHAPE;

namespace KIGFX
{
class VIEW;
class VIEW_OVERLAY;
}


/**
 * Live clearance check run while moving items: the Move tool's counterpart of the collision
 * highlighting the router shows when dragging.
 *
 * Init() indexes everything that is not moving once.  Run() then tests the moving copper at its
 * current position against the board's clearance, hole clearance and edge clearance rules.  Zone
 * fills are not tested: they are stale during a move and get refilled afterwards.
 */
class DRC_INTERACTIVE_CLEARANCE : public DRC_TEST_PROVIDER
{
public:
    DRC_INTERACTIVE_CLEARANCE( const std::shared_ptr<DRC_ENGINE>& aDRCEngine );

    virtual ~DRC_INTERACTIVE_CLEARANCE();

    /**
     * Start a move session.
     *
     * @param aMovingItems the items being moved; footprints and groups bring their children along.
     */
    void Init( BOARD* aBoard, const std::vector<BOARD_ITEM*>& aMovingItems );

    virtual bool Run() override;

    virtual const wxString GetName() const override
    {
        return wxT( "interactive_clearance" );
    }

    size_t GetViolationCount() const { return m_violations.size(); }

    /// Draw the violations found by the last Run().
    void UpdateConflicts( KIGFX::VIEW* aView );

    /// Remove the drawing; call when the move ends.
    void ClearConflicts( KIGFX::VIEW* aView );

private:
    struct VIOLATION
    {
        std::shared_ptr<SHAPE> m_shape;         ///< Copper or hole of the moving item
        std::shared_ptr<SHAPE> m_otherShape;    ///< Copper or hole it is too close to; null for an edge
        int                    m_clearance;     ///< Distance the rules ask for
    };

    /// The distance a rule asks for between two items, or -1 when its severity is ignore.
    int getClearance( DRC_CONSTRAINT_T aType, const BOARD_ITEM* a, const BOARD_ITEM* b,
                      PCB_LAYER_ID aLayer );

    /// True when two shapes are closer than a clearance from getClearance().
    bool collide( const SHAPE* aShape, const SHAPE* aOther, int aClearance ) const;

    void testCopper( BOARD_ITEM* aItem, const std::shared_ptr<SHAPE>& aShape, PCB_LAYER_ID aLayer,
                     BOARD_ITEM* aOther );

    void report( BOARD_ITEM* aItem, BOARD_ITEM* aOther, std::shared_ptr<SHAPE> aShape,
                 std::shared_ptr<SHAPE> aOtherShape, int aClearance );

private:
    std::vector<BOARD_ITEM*>                m_movingItems;  ///< Moving items with copper or holes
    std::vector<std::unique_ptr<PCB_SHAPE>> m_edges;        ///< Zero-width copies of the edge cuts

    DRC_RTREE m_copperTree;
    DRC_RTREE m_edgeTree;

    int m_largestClearance;
    int m_largestEdgeClearance;
    int m_epsilon;

    std::map<std::tuple<DRC_CONSTRAINT_T, const BOARD_ITEM*, const BOARD_ITEM*, PCB_LAYER_ID>, int>
            m_clearanceCache;

    std::vector<VIOLATION>                        m_violations;
    std::set<std::pair<BOARD_ITEM*, BOARD_ITEM*>> m_reported;

    std::shared_ptr<KIGFX::VIEW_OVERLAY> m_overlay;
};
