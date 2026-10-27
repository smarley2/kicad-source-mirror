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

/**
 * @file test_drc_interactive_clearance.cpp
 *
 * The live clearance check run while moving items must flag a foreign-net track crossing a moving
 * pad, and must clear once the pad has been moved away.
 *
 * Fixture (qa/data/pcbnew/issue24355/): netclass clearance 0.2 mm, one footprint at (150, 100) whose
 * pad "1" (PTH, 1 mm, no net) sits at (151.6, 100).
 */

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <pcbnew_utils/board_test_utils.h>

#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <netinfo.h>
#include <pcb_track.h>
#include <drc/drc_engine.h>
#include <drc/drc_interactive_clearance.h>
#include <settings/settings_manager.h>


struct DRC_INTERACTIVE_CLEARANCE_FIXTURE
{
    DRC_INTERACTIVE_CLEARANCE_FIXTURE() = default;

    ~DRC_INTERACTIVE_CLEARANCE_FIXTURE()
    {
        if( m_board )
        {
            m_board->SetProject( nullptr );
            m_board = nullptr;
        }
    }

    SETTINGS_MANAGER       m_settingsManager;
    std::unique_ptr<BOARD> m_board;
};


BOOST_FIXTURE_TEST_CASE( DRCInteractiveClearance, DRC_INTERACTIVE_CLEARANCE_FIXTURE )
{
    KI_TEST::LoadBoard( m_settingsManager, "issue24355/issue24355", m_board );

    BOOST_REQUIRE_MESSAGE( m_board, "Failed to load board issue24355" );
    BOOST_REQUIRE_EQUAL( m_board->Footprints().size(), 1 );

    NETINFO_ITEM* net = new NETINFO_ITEM( m_board.get(), wxT( "TEST" ), m_board->GetNetCount() );
    m_board->Add( net );

    // A vertical track straight through pad "1"
    PCB_TRACK* track = new PCB_TRACK( m_board.get() );
    track->SetStart( VECTOR2I( pcbIUScale.mmToIU( 151.6 ), pcbIUScale.mmToIU( 95 ) ) );
    track->SetEnd( VECTOR2I( pcbIUScale.mmToIU( 151.6 ), pcbIUScale.mmToIU( 105 ) ) );
    track->SetWidth( pcbIUScale.mmToIU( 0.2 ) );
    track->SetLayer( F_Cu );
    track->SetNet( net );
    m_board->Add( track );

    FOOTPRINT*                footprint = m_board->Footprints().front();
    DRC_INTERACTIVE_CLEARANCE checker( m_board->GetDesignSettings().m_DRCEngine );

    checker.Init( m_board.get(), { footprint } );

    checker.Run();
    BOOST_CHECK_GT( checker.GetViolationCount(), 0 );

    // 10 mm clear of the track and still well inside the board outline
    footprint->Move( VECTOR2I( pcbIUScale.mmToIU( -10 ), 0 ) );

    checker.Run();
    BOOST_CHECK_EQUAL( checker.GetViolationCount(), 0 );
}
