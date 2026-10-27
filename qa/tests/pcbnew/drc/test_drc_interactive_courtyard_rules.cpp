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
 * @file test_drc_interactive_courtyard_rules.cpp
 *
 * The live courtyard check run while moving footprints must apply courtyard_clearance rules,
 * not only flag courtyards that overlap.
 *
 * Fixture (qa/data/pcbnew/issue24264/): C1 and R1, 0603 footprints on the front.
 */

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <pcbnew_utils/board_test_utils.h>

#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <drc/drc_engine.h>
#include <drc/drc_interactive_courtyard_clearance.h>
#include <settings/settings_manager.h>

#include <filesystem>
#include <fstream>


struct DRC_INTERACTIVE_COURTYARD_RULES_FIXTURE
{
    DRC_INTERACTIVE_COURTYARD_RULES_FIXTURE() = default;

    ~DRC_INTERACTIVE_COURTYARD_RULES_FIXTURE()
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


BOOST_FIXTURE_TEST_CASE( DRCInteractiveCourtyardRules, DRC_INTERACTIVE_COURTYARD_RULES_FIXTURE )
{
    KI_TEST::TEMPORARY_DIRECTORY tmpDir( "kicad_interactive_courtyard_rules", "" );
    const std::filesystem::path  rulePath = tmpDir.GetPath() / "courtyard_rules.kicad_dru";

    {
        std::ofstream ruleFile( rulePath );
        ruleFile << "(version 1)\n"
                 << "(rule tall (constraint courtyard_clearance (min 3mm))\n"
                 << "    (condition \"A.Reference == 'R1' && B.Reference == 'C1'\"))\n";
    }

    KI_TEST::LoadBoard( m_settingsManager, "issue24264/issue24264", m_board );
    BOOST_REQUIRE_MESSAGE( m_board, "Failed to load board issue24264" );

    BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
    bds.m_DRCEngine->InitEngine( wxFileName( rulePath.string() ) );

    FOOTPRINT* r1 = m_board->FindFootprintByReference( wxT( "R1" ) );
    FOOTPRINT* c1 = m_board->FindFootprintByReference( wxT( "C1" ) );
    BOOST_REQUIRE( r1 && c1 );

    // R1 is the one moving, so the rule above only matches with the footprints swapped
    r1->SetOrientation( ANGLE_0 );
    r1->SetSelected();

    // Puts R1's courtyard aGap to the right of C1's, centred on it
    auto placeR1 =
            [&]( double aGap )
            {
                BOX2I r = r1->GetCourtyard( F_CrtYd ).BBox();
                BOX2I c = c1->GetCourtyard( F_CrtYd ).BBox();

                r1->Move( VECTOR2I( c.GetRight() + pcbIUScale.mmToIU( aGap ) - r.GetLeft(),
                                    c.GetCenter().y - r.GetCenter().y ) );
            };

    DRC_INTERACTIVE_COURTYARD_CLEARANCE checker( bds.m_DRCEngine );

    checker.Init( m_board.get() );
    checker.m_FpInMove.push_back( r1 );

    placeR1( 1.0 );
    checker.Run();
    BOOST_CHECK( checker.GetConflicts().count( r1 ) );
    BOOST_CHECK( checker.GetConflicts().count( c1 ) );

    placeR1( 5.0 );
    checker.Run();
    BOOST_CHECK( checker.GetConflicts().empty() );
}
