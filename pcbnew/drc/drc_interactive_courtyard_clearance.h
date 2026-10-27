/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2004-2019 Jean-Pierre Charras, jp.charras at wanadoo.fr
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */


#pragma once

#include <map>
#include <memory>
#include <set>
#include <tuple>

#include <drc/drc_test_provider.h>

namespace KIGFX
{
class VIEW_OVERLAY;
}


class DRC_INTERACTIVE_COURTYARD_CLEARANCE : public DRC_TEST_PROVIDER
{
public:
    DRC_INTERACTIVE_COURTYARD_CLEARANCE( const std::shared_ptr<DRC_ENGINE>& aDRCEngine ) :
            DRC_TEST_PROVIDER(),
            m_largestCourtyardClearance( 0 )
    {
        m_isRuleDriven = false;
        SetDRCEngine( aDRCEngine.get() );
    }

    virtual ~DRC_INTERACTIVE_COURTYARD_CLEARANCE () = default;

    void Init( BOARD* aBoard );

    virtual bool Run() override;

    virtual const wxString GetName() const override
    {
        return wxT( "interactive_courtyard_clearance" );
    }

    void UpdateConflicts( KIGFX::VIEW* aView, bool aHighlightMoved );
    void ClearConflicts( KIGFX::VIEW* aView );

    /// Items found in conflict by the last Run()
    const std::set<BOARD_ITEM*>& GetConflicts() const { return m_itemsInConflict; }

public:
    std::vector<FOOTPRINT*>   m_FpInMove;             // The list of moved footprints

private:
    void testCourtyardClearances();

    /// The courtyard_clearance rule between two footprints on a side, or -1 when it is ignored.
    int courtyardClearance( FOOTPRINT* aStatic, FOOTPRINT* aMoving, PCB_LAYER_ID aLayer );

private:
    int m_largestCourtyardClearance;

    std::set<BOARD_ITEM*>     m_itemsInConflict;      // The list of items in conflict
    std::vector<BOARD_ITEM*>  m_lastItemsInConflict;  // The list of items last highlighted

    /// Rule values per footprint pair and side, evaluated once per move
    std::map<std::tuple<FOOTPRINT*, FOOTPRINT*, PCB_LAYER_ID>, int> m_clearanceCache;

    /// Courtyards (footprint, courtyard layer, clearance) a moving one is too close to
    std::set<std::tuple<FOOTPRINT*, PCB_LAYER_ID, int>> m_keepOuts;

    std::shared_ptr<KIGFX::VIEW_OVERLAY> m_overlay;
};
