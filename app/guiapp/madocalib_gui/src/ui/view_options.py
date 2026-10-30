# Copyright (C) 2026 Cabinet Office, Japan, All rights reserved.
# Copyright (C) 2026 Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
# SPDX-License-Identifier: BSD-2-Clause
"""Options window UI for the madocalib GUI.

Builds tabs (Setting1/2.1/2.2, Output, Stats, Positions, Files, Misc)
and delegates load/save to `ui.actions_options`.

"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from core.runtime import get_conf_path, set_window_icon
from ui.actions_options import (
    on_click_file_blq,
    on_click_file_dcb,
    on_click_file_eop,
    on_click_file_geoid,
    on_click_file_ionosphere,
    on_click_file_satantfile1,
    on_click_file_satantfile2,
    on_click_file_start_position,
    on_click_load_conf,
    on_click_save_conf,
    on_click_save_overwrite_conf,
)
from ui.variables_options import VariablesOptions  # Provided by the project

WINDOW_SIZE = "640x400"


def option_window_views(main_window, variables_main) -> None:
    """Open the Options dialog and load the current .conf file."""
    option_window = tk.Toplevel(main_window)
    option_window.withdraw()
    variables_options = VariablesOptions(option_window)

    # Tab buttons
    variables_options.tab_key_active = None
    variables_options.default_bg_color = None
    variables_options.tab_buttons = {}
    variables_options.tab_buttons[0] = tk.Button(
        option_window,
        text="Setting1",
        bg="lightgray",
        command=lambda: activate_options_tab(0, variables_options),
    )
    variables_options.tab_buttons[1] = tk.Button(
        option_window,
        text="Setting2.1",
        bg="lightgray",
        command=lambda: activate_options_tab(1, variables_options),
    )
    variables_options.tab_buttons[2] = tk.Button(
        option_window,
        text="Setting2.2",
        bg="lightgray",
        command=lambda: activate_options_tab(2, variables_options),
    )
    variables_options.tab_buttons[3] = tk.Button(
        option_window,
        text="Output",
        bg="lightgray",
        command=lambda: activate_options_tab(3, variables_options),
    )
    variables_options.tab_buttons[4] = tk.Button(
        option_window,
        text="Stats",
        bg="lightgray",
        command=lambda: activate_options_tab(4, variables_options),
    )
    variables_options.tab_buttons[5] = tk.Button(
        option_window,
        text="Positions",
        bg="lightgray",
        command=lambda: activate_options_tab(5, variables_options),
    )
    variables_options.tab_buttons[6] = tk.Button(
        option_window,
        text="Files",
        bg="lightgray",
        command=lambda: activate_options_tab(6, variables_options),
    )
    variables_options.tab_buttons[7] = tk.Button(
        option_window,
        text="Misc",
        bg="lightgray",
        command=lambda: activate_options_tab(7, variables_options),
    )
    variables_options.default_bg_color = variables_options.tab_buttons[0].cget("bg")

    # IntVars for checkboxes
    variables_options.checkVar1 = tk.IntVar(value=0)
    variables_options.checkVar2 = tk.IntVar(value=0)
    variables_options.checkVar3 = tk.IntVar(value=0)
    variables_options.checkVar4 = tk.IntVar(value=0)
    variables_options.checkVar5 = tk.IntVar(value=0)
    variables_options.checkVar6 = tk.IntVar(value=0)

    variables_options.checkVarnav1 = tk.IntVar(value=0)
    variables_options.checkVarnav2 = tk.IntVar(value=0)
    variables_options.checkVarnav3 = tk.IntVar(value=0)
    variables_options.checkVarnav4 = tk.IntVar(value=0)
    variables_options.checkVarnav5 = tk.IntVar(value=0)

    variables_options.chk_pos2_baselen = tk.IntVar(value=0)
    variables_options.chk_position_anttype1 = tk.IntVar(value=0)
    variables_options.chk_position_anttype2 = tk.IntVar(value=0)

    # --- Setting1 ---
    variables_options.label_pos1_posmode = tk.Label(
        option_window, text="Positioning Mode"
    )
    variables_options.label_pos1_navsys = tk.Label(
        option_window, text="Navigation Systems"
    )
    variables_options.label_pos1_frequency_soltype = tk.Label(
        option_window, text="Frequencies / Filter Type"
    )
    variables_options.label_pos1_elmask_snrmask = tk.Label(
        option_window, text="Elevation Mask (°) / SNR Mask (dBHz)"
    )
    variables_options.label_pos1_dynamics_tidecorr = tk.Label(
        option_window, text="Rec Dynamics / Earth Tides Corrections"
    )
    variables_options.label_pos1_ionoopt = tk.Label(
        option_window, text="Ionosphere Corrections"
    )
    variables_options.label_pos1_tropopt = tk.Label(
        option_window, text="Troposphere Corrections"
    )
    variables_options.label_pos1_sateph = tk.Label(
        option_window, text="Satellite Ephemeris"
    )
    variables_options.label_pos1_exclsats = tk.Label(
        option_window, text="Excluded Satellites (+PRN: Included)"
    )

    variables_options.combo_pos1_posmode = ttk.Combobox(
        option_window,
        values=[
            "single",
            "ppp-kine",
            "ppp-static",
            "ppp-fixed",
        ],
        width=40,
    )
    # Checkboxes (navsys)
    variables_options.chk_pos1_navsys1 = tk.Checkbutton(
        option_window, text="GPS", variable=variables_options.checkVarnav1
    )
    variables_options.chk_pos1_navsys4 = tk.Checkbutton(
        option_window, text="QZSS", variable=variables_options.checkVarnav4
    )
    variables_options.chk_pos1_navsys2 = tk.Checkbutton(
        option_window, text="GLONASS", variable=variables_options.checkVarnav2
    )
    variables_options.chk_pos1_navsys3 = tk.Checkbutton(
        option_window, text="Galileo", variable=variables_options.checkVarnav3
    )
    variables_options.chk_pos1_navsys5 = tk.Checkbutton(
        option_window, text="BDS", variable=variables_options.checkVarnav5
    )
    variables_options.combo_pos1_frequency_soltype1 = ttk.Combobox(
        option_window, values=["l1", "l1+2", "l1+2+3", "l1+2+3+4"], width=17
    )
    variables_options.combo_pos1_frequency_soltype2 = ttk.Combobox(
        option_window, values=["forward"], width=17
    )
    variables_options.combo_pos1_elmask_snrmask1 = ttk.Combobox(
        option_window,
        values=[
            "0",
            "5",
            "10",
            "15",
            "20",
            "25",
            "30",
            "35",
            "40",
            "45",
            "50",
            "55",
            "60",
            "65",
            "70",
        ],
        width=17,
    )
    variables_options.combo_pos1_elmask_snrmask2 = ttk.Combobox(
        option_window, values=["off", "on"], width=17
    )
    variables_options.combo_pos1_dynamics_tidecorr1 = ttk.Combobox(
        option_window, values=["off", "on"], width=17
    )
    variables_options.combo_pos1_dynamics_tidecorr2 = ttk.Combobox(
        option_window, values=["off", "on", "otl"], width=17
    )
    variables_options.combo_pos1_ionoopt = ttk.Combobox(
        option_window,
        values=[
            "off",
            "brdc",
            "sbas",
            "dual-freq",
            "est-stec",
        ],
        width=40,
    )
    variables_options.combo_pos1_tropopt = ttk.Combobox(
        option_window,
        values=["off", "saas", "sbas", "est-ztd", "est-ztdgrad"],
        width=40,
    )
    variables_options.combo_pos1_sateph = ttk.Combobox(
        option_window,
        values=["brdc", "precise", "brdc+sbas", "brdc+ssrapc", "brdc+ssrcom"],
        width=40,
    )
    variables_options.chk_pos1_posopt1 = tk.Checkbutton(
        option_window, text="Sat PCV", variable=variables_options.checkVar1
    )
    variables_options.chk_pos1_posopt2 = tk.Checkbutton(
        option_window, text="Rec PCV", variable=variables_options.checkVar2
    )
    variables_options.chk_pos1_posopt3 = tk.Checkbutton(
        option_window, text="PhWU", variable=variables_options.checkVar3
    )
    variables_options.chk_pos1_posopt4 = tk.Checkbutton(
        option_window, text="Rej Ecl", variable=variables_options.checkVar4
    )
    variables_options.chk_pos1_posopt5 = tk.Checkbutton(
        option_window, text="RAIM FDE", variable=variables_options.checkVar5
    )
    variables_options.chk_pos1_posopt6 = tk.Checkbutton(
        option_window, text="Clock jump", variable=variables_options.checkVar6
    )
    variables_options.entry_pos1_exclsats = tk.Entry(option_window, width=40)

    # --- Setting2.1 ---
    variables_options.label_pos2_armode = tk.Label(option_window, text="AR Mode")
    variables_options.label_pos2_arsys = tk.Label(
        option_window, text="Integer Ambiguity Res (GPS/QZS/GLO/GAL/BDS)"
    )
    variables_options.label_pos2_ionocorr = tk.Label(
        option_window, text="Ionospheric Correction"
    )
    variables_options.label_pos2_arthres = tk.Label(
        option_window, text="Min Ratio to Fix Ambiguity"
    )
    variables_options.label_pos2_arth1_2 = tk.Label(
        option_window, text="Min Confidence / Max FCB to Fix Amb"
    )
    variables_options.label_pos2_arlockcnt_arelmask = tk.Label(
        option_window, text="Min Lock / Elevation (°) to Fix Amb"
    )
    variables_options.label_pos2_arminfix_elmaskhold = tk.Label(
        option_window, text="Min Fix / Elevation (°) to Hold Amb"
    )
    variables_options.label_pos2_aroutcnt_slipthres = tk.Label(
        option_window, text="Outage to Reset Amb / Slip Thres (m)"
    )
    variables_options.label_pos2_maxage_syncsol = tk.Label(
        option_window, text="Max Age of Diff (s) / Sync Solution"
    )
    variables_options.label_pos2_rejionno_rejgdop = tk.Label(
        option_window, text="Reject Threshold of GDOP/Innov (m)"
    )
    variables_options.label_pos2_niter = tk.Label(
        option_window, text="Max # of AR Iter /# of Filter Iter"
    )

    variables_options.combo_pos2_armode = ttk.Combobox(
        option_window, values=["off", "continuous", "fix-and-hold"], width=13
    )
    variables_options.combo_pos2_arsys1 = ttk.Combobox(
        option_window, values=["off", "on"], width=3
    )  # GPS
    variables_options.combo_pos2_arsys4 = ttk.Combobox(
        option_window, values=["off", "on"], width=3
    )  # QZS
    variables_options.combo_pos2_arsys2 = ttk.Combobox(
        option_window, values=["off", "on"], width=3
    )  # GLO
    variables_options.combo_pos2_arsys3 = ttk.Combobox(
        option_window, values=["off", "on"], width=3
    )  # GAL
    variables_options.combo_pos2_arsys5 = ttk.Combobox(
        option_window, values=["off", "on"], width=3
    )  # BDS

    variables_options.combo_pos2_ionocorr1 = ttk.Combobox(
        option_window, values=["off", "on"], width=37
    )
    variables_options.entry_pos2_arthres = tk.Entry(option_window, width=40)
    variables_options.entry_pos2_arth1_21 = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_arth1_22 = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_arlockcnt_arelmask1 = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_arlockcnt_arelmask2 = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_arminfix_elmaskhold1 = tk.Entry(
        option_window, width=17
    )
    variables_options.entry_pos2_arminfix_elmaskhold2 = tk.Entry(
        option_window, width=17
    )
    variables_options.entry_pos2_aroutcnt_slipthres1 = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_aroutcnt_slipthres2 = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_maxage_syncsol1 = tk.Entry(option_window, width=17)
    variables_options.combo_pos2_maxage_syncsol2 = ttk.Combobox(
        option_window, values=["off", "on"], width=14
    )
    variables_options.entry_pos2_rejionno_rejgdop1 = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_rejionno_rejgdop2 = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_AR_Filter_Iter = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_niter = tk.Entry(option_window, width=17)
    variables_options.chk_pos2_baselen = tk.Checkbutton(
        option_window,
        text="Baseline Length Constraint (m)",
        variable=variables_options.chk_pos2_baselen,
    )
    variables_options.entry_pos2_baselen = tk.Entry(option_window, width=17)
    variables_options.entry_pos2_basesig = tk.Entry(option_window, width=17)

    # --- Setting2.2 ---
    variables_options.label_pos2_siggps = tk.Label(
        option_window, text="GPS Signal Combination"
    )
    variables_options.label_pos2_sigqzs = tk.Label(
        option_window, text="QZSS Signal Combination"
    )
    variables_options.label_pos2_siggal = tk.Label(
        option_window, text="Galileo Signal Combination"
    )
    variables_options.label_pos2_sigbds2 = tk.Label(
        option_window, text="BeiDou-2 Signal Combination"
    )
    variables_options.label_pos2_sigbds3 = tk.Label(
        option_window, text="BeiDou-3 Signal Combination"
    )

    variables_options.combo_pos2_siggps = ttk.Combobox(
        option_window, values=["L1/L2", "L1/L5", "L1/L2/L5"], width=25
    )
    variables_options.combo_pos2_sigqzs = ttk.Combobox(
        option_window, values=["L1/L5", "L1/L2", "L1/L5/L2"], width=25
    )
    variables_options.combo_pos2_siggal = ttk.Combobox(
        option_window,
        values=["E1/E5a", "E1/E5b", "E1/E5a/E5b/E6", "E1/E5a/E6/E5b"],
        width=25,
    )
    variables_options.combo_pos2_sigbds2 = ttk.Combobox(
        option_window, values=["B1I/B3I", "B1I/B2I", "B1I/B3I/B2I"], width=25
    )
    variables_options.combo_pos2_sigbds3 = ttk.Combobox(
        option_window, values=["B1I/B3I", "B1I/B2a", "B1I/B3I/B2a"], width=25
    )

    # BDS-2 satellites in PRN 1-18 (pos2-bds2prn)
    variables_options.label_pos2_bds2prn = tk.Label(
        option_window, text="BeiDou-2 Satellites in PRN 1-18"
    )
    variables_options.entry_pos2_bds2prn = tk.Entry(option_window, width=27)

    # BDS-3 IGSO satellites (pos2-bds3igso)
    variables_options.label_pos2_bds3igso = tk.Label(
        option_window, text="BeiDou-3 IGSO Satellites"
    )
    variables_options.entry_pos2_bds3igso = tk.Entry(option_window, width=27)

    # --- Output ---
    variables_options.label_out_solformat = tk.Label(
        option_window, text="Solution Format"
    )
    variables_options.label_out_outhead_outopt = tk.Label(
        option_window, text="Output Header / Processing Options / Velocity"
    )
    variables_options.label_out_timesys_timeform_timendec = tk.Label(
        option_window, text="Time Format / # of Decimals"
    )
    variables_options.label_out_degform = tk.Label(
        option_window, text="Latitude Longitude Format"
    )
    variables_options.label_out_fieldsep = tk.Label(
        option_window, text="Field Separator"
    )
    variables_options.label_out_height = tk.Label(option_window, text="Height")
    variables_options.label_out_geoid = tk.Label(option_window, text="Geoid Model")
    variables_options.label_out_solstatic = tk.Label(
        option_window, text="Solution for Static Mode"
    )
    variables_options.label_out_nmeaintv = tk.Label(
        option_window, text="NMEA Interval (s) RMC/GGA, GSA/GSV"
    )
    variables_options.label_out_outstat = tk.Label(
        option_window, text="Output Solution Status"
    )

    variables_options.combo_outsolformat = ttk.Combobox(
        option_window,
        values=["Lat/Lon/Height", "X/Y/Z-ECEF", "E/N/U-Baseline", "NMEA 0183"],
        width=37,
    )
    variables_options.combo_out_outhead_outopt1 = ttk.Combobox(
        option_window, values=["off", "on"], width=7
    )
    variables_options.combo_out_outhead_outopt2 = ttk.Combobox(
        option_window, values=["off", "on"], width=7
    )
    variables_options.combo_out_outvel = ttk.Combobox(
        option_window, values=["off", "on"], width=7
    )
    variables_options.combo_out_timesys_timeform_timendec1 = ttk.Combobox(
        option_window,
        values=["ww ssss GPST", "hh:mm:ss GPST", "hh:mm:ss UTC", "hh:mm:ss JST"],
        width=25,
    )
    variables_options.entry_out_timesys_timeform_timendec2 = tk.Entry(
        option_window, width=10
    )  # decimals
    variables_options.combo_out_degform = ttk.Combobox(
        option_window, values=["ddd.ddddddd", "ddd mm ss.sss"], width=37
    )
    variables_options.entry_out_fieldsep = tk.Entry(option_window, width=40)
    variables_options.combo_out_height = ttk.Combobox(
        option_window, values=["ellipsoidal", "geodetic"], width=37
    )
    variables_options.combo_out_geoid = ttk.Combobox(
        option_window,
        values=["internal", "egm96", "egm08_2.5", "egm08_1", "gsi2000"],
        width=37,
    )
    variables_options.combo_out_solstatic = ttk.Combobox(
        option_window, values=["all", "single"], width=37
    )
    variables_options.entry_out_nmeaintv1 = tk.Entry(option_window, width=17)
    variables_options.entry_out_nmeaintv2 = tk.Entry(option_window, width=17)
    variables_options.combo_out_outstat = ttk.Combobox(
        option_window, values=["off", "state", "residual"], width=14
    )

    # --- Stats ---
    variables_options.frame_stat_1 = tk.LabelFrame(
        option_window, text="Measurement Errors (1-sigma)", width=610, height=145
    )
    variables_options.frame_stat_2 = tk.LabelFrame(
        option_window,
        text="Process Noises (1-sigma/sqrt(s))",
        width=610,
        height=165,
    )

    variables_options.label_stat_eratio = tk.Label(
        option_window, text="Code/Carrier-Phase Error Ratio L1/L2"
    )
    variables_options.label_stat_prnaccelh_prnaccelv = tk.Label(
        option_window, text="Receiver Accel Horizon/Vertical (m/s2)"
    )
    variables_options.label_stat_errphase = tk.Label(
        option_window, text="Carrier-Phase Error a+b/sinEl (m)"
    )
    variables_options.label_stat_errphaseb1 = tk.Label(
        option_window, text="Carrier-Phase Error/Baseline (m/10km)"
    )
    variables_options.label_stat_errdoppler = tk.Label(
        option_window, text="Doppler Frequency (Hz)"
    )
    variables_options.label_stat_uraratio = tk.Label(
        option_window, text="Ratio for External URA"
    )
    variables_options.label_stat_prnbias = tk.Label(
        option_window, text="Carrier-Phase Bias (cycle)"
    )
    variables_options.label_stat_prniono = tk.Label(
        option_window, text="Vertical Ionospheric Delay (m/10km)"
    )
    variables_options.label_stat_prntrop = tk.Label(
        option_window, text="Zenith Tropospheric Delay (m)"
    )
    variables_options.label_stat_prnifb = tk.Label(
        option_window, text="Inter Frequency Bias (m)"
    )
    variables_options.label_stat_prnpos = tk.Label(
        option_window, text="Position Process Noise (m)"
    )
    variables_options.label_stat_clktab = tk.Label(
        option_window, text="Satellite Clock Stability (s/s)"
    )

    variables_options.entry_stat_eratio1 = tk.Entry(option_window, width=17)
    variables_options.entry_stat_eratio2 = tk.Entry(option_window, width=17)
    variables_options.entry_stat_errphase1 = tk.Entry(option_window, width=17)
    variables_options.entry_stat_errphase2 = tk.Entry(option_window, width=17)
    variables_options.entry_stat_errphaseb1 = tk.Entry(option_window, width=40)
    variables_options.entry_stat_errdoppler = tk.Entry(option_window, width=40)
    variables_options.entry_stat_uraratio = tk.Entry(option_window, width=40)
    variables_options.entry_stat_prnaccelh_prnaccelv1 = tk.Entry(
        option_window, width=17
    )
    variables_options.entry_stat_prnaccelh_prnaccelv2 = tk.Entry(
        option_window, width=17
    )
    variables_options.entry_stat_prnbias = tk.Entry(option_window, width=40)
    variables_options.entry_stat_prniono = tk.Entry(option_window, width=40)
    variables_options.entry_stat_prntrop = tk.Entry(option_window, width=40)
    variables_options.entry_stat_prnifb = tk.Entry(option_window, width=40)
    variables_options.entry_stat_prnpos = tk.Entry(option_window, width=40)
    variables_options.entry_stat_clktab = tk.Entry(option_window, width=40)

    # --- Positions ---
    variables_options.frame_position_1 = tk.LabelFrame(
        option_window, text="Rover", width=610, height=135
    )
    variables_options.frame_position_2 = tk.LabelFrame(
        option_window, text="Base Station", width=610, height=145
    )

    variables_options.label_position_antde1 = tk.Label(
        option_window, text="Delta-E/N/U (m)"
    )
    variables_options.label_position_antde2 = tk.Label(
        option_window, text="Delta-E/N/U (m)"
    )
    variables_options.label_position_file_staposfile = tk.Label(
        option_window, text="Station Position File"
    )

    variables_options.combo_position_postype1 = ttk.Combobox(
        option_window,
        values=["llh", "xyz", "single", "posfile", "rinexhead", "rtcm", "raw"],
        width=40,
    )
    variables_options.entry_position_pos11 = tk.Entry(option_window, width=30)
    variables_options.entry_position_pos21 = tk.Entry(option_window, width=30)
    variables_options.entry_position_pos31 = tk.Entry(option_window, width=30)
    variables_options.chk_position_anttype1 = tk.IntVar(value=0)
    variables_options.chk_position_anttype1 = tk.Checkbutton(
        option_window,
        text="Antenna Type (*:Auto)",
        variable=variables_options.chk_position_anttype1,
    )
    variables_options.entry_position_anttype1 = tk.Entry(option_window, width=55)
    variables_options.entry_position_antdele1 = tk.Entry(option_window, width=10)
    variables_options.entry_position_antdeln1 = tk.Entry(option_window, width=10)
    variables_options.entry_position_antdelu1 = tk.Entry(option_window, width=10)

    variables_options.combo_position_postype2 = ttk.Combobox(
        option_window,
        values=["llh", "xyz", "single", "posfile", "rinexhead", "rtcm", "raw"],
        width=40,
    )
    variables_options.entry_position_pos12 = tk.Entry(option_window, width=30)
    variables_options.entry_position_pos22 = tk.Entry(option_window, width=30)
    variables_options.entry_position_pos32 = tk.Entry(option_window, width=30)
    variables_options.chk_position_anttype2 = tk.IntVar(value=0)
    variables_options.chk_position_anttype2 = tk.Checkbutton(
        option_window,
        text="Antenna Type (*:Auto)",
        variable=variables_options.chk_position_anttype2,
    )
    variables_options.entry_position_anttype2 = tk.Entry(option_window, width=55)
    variables_options.entry_position_antdele2 = tk.Entry(option_window, width=10)
    variables_options.entry_position_antdeln2 = tk.Entry(option_window, width=10)
    variables_options.entry_position_antdelu2 = tk.Entry(option_window, width=10)

    variables_options.entry_position_staposfile = tk.Entry(option_window, width=92)
    variables_options.btn_position_staposfile = tk.Button(
        option_window,
        text="...",
        bg="lightgray",
        command=lambda: on_click_file_start_position(
            option_window, variables_options.entry_position_staposfile
        ),
    )

    # --- Files ---
    variables_options.label_file_satantfile = tk.Label(
        option_window, text="Satellite/Receiver Antenna PCV File ANTEX/NGS PCV"
    )
    variables_options.label_file_geoidfile = tk.Label(
        option_window, text="Geoid Data File"
    )
    variables_options.label_file_dcbfile = tk.Label(option_window, text="DCB Data File")
    variables_options.label_file_eopfile = tk.Label(option_window, text="EOP Data File")
    variables_options.label_file_blqfile = tk.Label(option_window, text="OTL BLQ File")
    variables_options.label_file_ionofile = tk.Label(
        option_window, text="Ionosphere Data File"
    )

    variables_options.entry_file_satantfile1 = tk.Entry(option_window, width=92)
    variables_options.btn_file_satantfile1 = tk.Button(
        option_window,
        text="...",
        bg="lightgray",
        command=lambda: on_click_file_satantfile1(
            option_window, variables_options.entry_file_satantfile1
        ),
    )
    variables_options.entry_file_satantfile2 = tk.Entry(option_window, width=92)
    variables_options.btn_file_satantfile2 = tk.Button(
        option_window,
        text="...",
        bg="lightgray",
        command=lambda: on_click_file_satantfile2(
            option_window, variables_options.entry_file_satantfile2
        ),
    )
    variables_options.entry_file_geoidfile = tk.Entry(option_window, width=92)
    variables_options.btn_file_geoidfile = tk.Button(
        option_window,
        text="...",
        bg="lightgray",
        command=lambda: on_click_file_geoid(
            option_window, variables_options.entry_file_geoidfile
        ),
    )
    variables_options.entry_file_dcbfile = tk.Entry(option_window, width=92)
    variables_options.btn_file_dcbfile = tk.Button(
        option_window,
        text="...",
        bg="lightgray",
        command=lambda: on_click_file_dcb(
            option_window, variables_options.entry_file_dcbfile
        ),
    )
    variables_options.entry_file_eopfile = tk.Entry(option_window, width=92)
    variables_options.btn_file_eopfile = tk.Button(
        option_window,
        text="...",
        bg="lightgray",
        command=lambda: on_click_file_eop(
            option_window, variables_options.entry_file_eopfile
        ),
    )
    variables_options.entry_file_blqfile = tk.Entry(option_window, width=92)
    variables_options.btn_file_blqfile = tk.Button(
        option_window,
        text="...",
        bg="lightgray",
        command=lambda: on_click_file_blq(
            option_window, variables_options.entry_file_blqfile
        ),
    )
    variables_options.entry_file_ionofile = tk.Entry(option_window, width=92)
    variables_options.btn_file_ionofile = tk.Button(
        option_window,
        text="...",
        bg="lightgray",
        command=lambda: on_click_file_ionosphere(
            option_window, variables_options.entry_file_ionofile
        ),
    )

    # --- Misc ---
    variables_options.label_misc_timeinterp = tk.Label(
        option_window, text="Time Interpolation of Base Station Data"
    )
    variables_options.label_misc_sbasatsel = tk.Label(
        option_window, text="SBAS Satellite Selection (0:All)"
    )
    variables_options.label_misc_rnxopt1 = tk.Label(
        option_window, text="RINEX Opt (Rover)"
    )
    variables_options.label_misc_rnxopt2 = tk.Label(
        option_window, text="RINEX Opt (Base)"
    )
    variables_options.label_misc_pppopt = tk.Label(option_window, text="PPP Opt")
    variables_options.label_misc_rtcopt = tk.Label(option_window, text="RTCM Opt")

    variables_options.combo_misc_timeinterp = ttk.Combobox(
        option_window, width=40, values=["off", "on"]
    )
    variables_options.entry_misc_sbasatsel = tk.Entry(option_window, width=43)
    variables_options.entry_misc_rnxopt1 = tk.Entry(option_window, width=65)
    variables_options.entry_misc_rnxopt2 = tk.Entry(option_window, width=65)
    variables_options.entry_misc_pppopt = tk.Entry(option_window, width=65)
    variables_options.entry_misc_rtcopt = tk.Entry(option_window, width=65)

    filepath = on_click_load_conf(option_window, variables_options, get_conf_path())

    option_window.geometry(WINDOW_SIZE)
    option_window.grab_set()  # modal
    option_window.focus_set()  # focus
    option_window.title("Options : " + get_conf_path())
    set_window_icon(option_window)
    if filepath is None:
        option_window.title("Options : Please load the conf file")

    # Footer buttons
    btn_command1 = tk.Button(
        option_window,
        text="Load",
        bg="lightgray",
        command=lambda: on_click_load_conf(option_window, variables_options, ""),
    )
    btn_command2 = tk.Button(
        option_window,
        text="Save As",
        bg="lightgray",
        command=lambda: on_click_save_conf(option_window, variables_options, ""),
    )
    btn_command3 = tk.Button(
        option_window,
        text="Save",
        bg="lightgray",
        command=lambda: on_click_save_overwrite_conf(option_window, variables_options),
    )
    btn_command4 = tk.Button(
        option_window, text="Cancel", bg="lightgray", command=option_window.destroy
    )

    # Header tab buttons
    variables_options.tab_buttons[0].place(x=0, y=0, width=70)
    variables_options.tab_buttons[1].place(x=70, y=0, width=70)
    variables_options.tab_buttons[2].place(x=140, y=0, width=70)
    variables_options.tab_buttons[3].place(x=210, y=0, width=70)
    variables_options.tab_buttons[4].place(x=280, y=0, width=70)
    variables_options.tab_buttons[5].place(x=350, y=0, width=70)
    variables_options.tab_buttons[6].place(x=420, y=0, width=70)
    variables_options.tab_buttons[7].place(x=490, y=0, width=70)

    btn_command1.place(x=20, y=360, width=90)
    btn_command2.place(x=130, y=360, width=90)
    btn_command3.place(x=410, y=360, width=90)
    btn_command4.place(x=520, y=360, width=90)

    # Default screen
    activate_options_tab(0, variables_options)
    option_window.deiconify()


def activate_options_tab(N: int, variables_options) -> None:
    """Activate a tab and show its content."""

    def update_tab_button_styles(N, variables_options) -> None:
        normal_cfg = dict(
            bg=variables_options.default_bg_color, fg="black", relief="raised"
        )
        active_cfg = dict(bg="#e2e9f1", fg="black", relief="sunken")
        for index, btn in variables_options.tab_buttons.items():
            if index == N:
                btn.configure(**active_cfg)
            else:
                btn.configure(**normal_cfg)

    activation = [0, 0, 0, 0, 0, 0, 0, 0]
    activation[N] = 1
    setting1(activation[0], variables_options)
    setting2_1(activation[1], variables_options)
    setting2_2(activation[2], variables_options)
    output(activation[3], variables_options)
    stats(activation[4], variables_options)
    positions(activation[5], variables_options)
    files(activation[6], variables_options)
    misc(activation[7], variables_options)
    update_tab_button_styles(N, variables_options)


def setting1(N: int, variables_options) -> None:
    """Show or hide widgets for the Setting1 tab."""
    if N == 0:
        variables_options.label_pos1_posmode.place_forget()
        variables_options.label_pos1_navsys.place_forget()
        variables_options.label_pos1_frequency_soltype.place_forget()
        variables_options.label_pos1_elmask_snrmask.place_forget()
        variables_options.label_pos1_dynamics_tidecorr.place_forget()
        variables_options.label_pos1_ionoopt.place_forget()
        variables_options.label_pos1_tropopt.place_forget()
        variables_options.label_pos1_sateph.place_forget()
        variables_options.label_pos1_exclsats.place_forget()
        variables_options.chk_pos1_posopt1.place_forget()
        variables_options.chk_pos1_posopt2.place_forget()
        variables_options.chk_pos1_posopt3.place_forget()
        variables_options.chk_pos1_posopt4.place_forget()
        variables_options.chk_pos1_posopt5.place_forget()
        variables_options.chk_pos1_posopt6.place_forget()
        variables_options.chk_pos1_navsys1.place_forget()
        variables_options.chk_pos1_navsys2.place_forget()
        variables_options.chk_pos1_navsys3.place_forget()
        variables_options.chk_pos1_navsys4.place_forget()
        variables_options.chk_pos1_navsys5.place_forget()
        variables_options.combo_pos1_posmode.place_forget()
        variables_options.combo_pos1_frequency_soltype1.place_forget()
        variables_options.combo_pos1_frequency_soltype2.place_forget()
        variables_options.combo_pos1_elmask_snrmask1.place_forget()
        variables_options.combo_pos1_elmask_snrmask2.place_forget()
        variables_options.combo_pos1_dynamics_tidecorr1.place_forget()
        variables_options.combo_pos1_dynamics_tidecorr2.place_forget()
        variables_options.combo_pos1_ionoopt.place_forget()
        variables_options.combo_pos1_tropopt.place_forget()
        variables_options.combo_pos1_sateph.place_forget()
        variables_options.entry_pos1_exclsats.place_forget()
    else:
        variables_options.label_pos1_posmode.place(x=20, y=40)
        variables_options.label_pos1_navsys.place(x=20, y=75)
        variables_options.label_pos1_frequency_soltype.place(x=20, y=110)
        variables_options.label_pos1_elmask_snrmask.place(x=20, y=140)
        variables_options.label_pos1_dynamics_tidecorr.place(x=20, y=170)
        variables_options.label_pos1_ionoopt.place(x=20, y=200)
        variables_options.label_pos1_tropopt.place(x=20, y=230)
        variables_options.label_pos1_sateph.place(x=20, y=260)
        variables_options.label_pos1_exclsats.place(x=20, y=325)
        variables_options.combo_pos1_posmode.place(x=340, y=40)
        variables_options.chk_pos1_navsys1.place(x=250, y=75)  # GPS
        variables_options.chk_pos1_navsys4.place(x=320, y=75)  # QZSS
        variables_options.chk_pos1_navsys2.place(x=390, y=75)  # GLONASS
        variables_options.chk_pos1_navsys3.place(x=480, y=75)  # Galileo
        variables_options.chk_pos1_navsys5.place(x=550, y=75)  # BDS
        variables_options.combo_pos1_frequency_soltype1.place(x=340, y=110)
        variables_options.combo_pos1_frequency_soltype2.place(x=477, y=110)
        variables_options.combo_pos1_elmask_snrmask1.place(x=340, y=140)
        variables_options.combo_pos1_elmask_snrmask2.place(x=477, y=140)
        variables_options.combo_pos1_dynamics_tidecorr1.place(x=340, y=170)
        variables_options.combo_pos1_dynamics_tidecorr2.place(x=477, y=170)
        variables_options.combo_pos1_ionoopt.place(x=340, y=200)
        variables_options.combo_pos1_tropopt.place(x=340, y=230)
        variables_options.combo_pos1_sateph.place(x=340, y=260)
        variables_options.chk_pos1_posopt1.place(x=20, y=290)
        variables_options.chk_pos1_posopt2.place(x=100, y=290)
        variables_options.chk_pos1_posopt3.place(x=180, y=290)
        variables_options.chk_pos1_posopt4.place(x=260, y=290)
        variables_options.chk_pos1_posopt5.place(x=340, y=290)
        variables_options.chk_pos1_posopt6.place(x=420, y=290)
        variables_options.entry_pos1_exclsats.place(x=340, y=325)


def setting2_1(N: int, variables_options) -> None:
    """Show or hide widgets for the Setting2.1 tab."""
    if N == 0:
        variables_options.label_pos2_armode.place_forget()
        variables_options.label_pos2_arsys.place_forget()
        variables_options.label_pos2_ionocorr.place_forget()
        variables_options.label_pos2_arthres.place_forget()
        variables_options.label_pos2_arth1_2.place_forget()
        variables_options.label_pos2_arlockcnt_arelmask.place_forget()
        variables_options.label_pos2_arminfix_elmaskhold.place_forget()
        variables_options.label_pos2_aroutcnt_slipthres.place_forget()
        variables_options.label_pos2_maxage_syncsol.place_forget()
        variables_options.label_pos2_rejionno_rejgdop.place_forget()
        variables_options.label_pos2_niter.place_forget()
        variables_options.combo_pos2_armode.place_forget()
        variables_options.combo_pos2_arsys1.place_forget()
        variables_options.combo_pos2_arsys4.place_forget()
        variables_options.combo_pos2_arsys2.place_forget()
        variables_options.combo_pos2_arsys3.place_forget()
        variables_options.combo_pos2_arsys5.place_forget()
        variables_options.combo_pos2_ionocorr1.place_forget()
        variables_options.entry_pos2_arthres.place_forget()
        variables_options.entry_pos2_arth1_21.place_forget()
        variables_options.entry_pos2_arth1_22.place_forget()
        variables_options.entry_pos2_arlockcnt_arelmask1.place_forget()
        variables_options.entry_pos2_arlockcnt_arelmask2.place_forget()
        variables_options.entry_pos2_arminfix_elmaskhold1.place_forget()
        variables_options.entry_pos2_arminfix_elmaskhold2.place_forget()
        variables_options.entry_pos2_aroutcnt_slipthres1.place_forget()
        variables_options.entry_pos2_aroutcnt_slipthres2.place_forget()
        variables_options.entry_pos2_maxage_syncsol1.place_forget()
        variables_options.combo_pos2_maxage_syncsol2.place_forget()
        variables_options.entry_pos2_rejionno_rejgdop1.place_forget()
        variables_options.entry_pos2_rejionno_rejgdop2.place_forget()
        variables_options.entry_pos2_niter.place_forget()
        variables_options.entry_pos2_AR_Filter_Iter.place_forget()
        variables_options.entry_pos2_baselen.place_forget()
        variables_options.entry_pos2_basesig.place_forget()
        variables_options.chk_pos2_baselen.place_forget()
    else:
        variables_options.label_pos2_armode.place(x=20, y=40)
        variables_options.label_pos2_arsys.place(x=20, y=70)
        variables_options.label_pos2_ionocorr.place(x=20, y=100)
        variables_options.label_pos2_arthres.place(x=20, y=125)
        variables_options.label_pos2_arth1_2.place(x=20, y=150)
        variables_options.label_pos2_arlockcnt_arelmask.place(x=20, y=175)
        variables_options.label_pos2_arminfix_elmaskhold.place(x=20, y=200)
        variables_options.label_pos2_aroutcnt_slipthres.place(x=20, y=225)
        variables_options.label_pos2_maxage_syncsol.place(x=20, y=250)
        variables_options.label_pos2_rejionno_rejgdop.place(x=20, y=275)
        variables_options.label_pos2_niter.place(x=20, y=300)
        variables_options.combo_pos2_armode.place(x=340, y=40)
        variables_options.combo_pos2_arsys1.place(x=340, y=70)  # GPS
        variables_options.combo_pos2_arsys4.place(x=391, y=70)  # QZS
        variables_options.combo_pos2_arsys2.place(x=442, y=70)  # GLO
        variables_options.combo_pos2_arsys3.place(x=493, y=70)  # GAL
        variables_options.combo_pos2_arsys5.place(x=544, y=70)  # BDS
        variables_options.combo_pos2_ionocorr1.place(x=340, y=100)
        variables_options.entry_pos2_arthres.place(x=340, y=125)
        variables_options.entry_pos2_arth1_21.place(x=340, y=150)
        variables_options.entry_pos2_arth1_22.place(x=477, y=150)
        variables_options.entry_pos2_arlockcnt_arelmask1.place(x=340, y=175)
        variables_options.entry_pos2_arlockcnt_arelmask2.place(x=477, y=175)
        variables_options.entry_pos2_arminfix_elmaskhold1.place(x=340, y=200)
        variables_options.entry_pos2_arminfix_elmaskhold2.place(x=477, y=200)
        variables_options.entry_pos2_aroutcnt_slipthres1.place(x=340, y=225)
        variables_options.entry_pos2_aroutcnt_slipthres2.place(x=477, y=225)
        variables_options.entry_pos2_maxage_syncsol1.place(x=340, y=250)
        variables_options.combo_pos2_maxage_syncsol2.place(x=477, y=250)
        variables_options.entry_pos2_rejionno_rejgdop1.place(x=340, y=275)
        variables_options.entry_pos2_rejionno_rejgdop2.place(x=477, y=275)
        variables_options.entry_pos2_AR_Filter_Iter.place(x=340, y=300)
        variables_options.entry_pos2_niter.place(x=477, y=300)
        variables_options.entry_pos2_baselen.place(x=340, y=330)
        variables_options.entry_pos2_basesig.place(x=477, y=330)
        variables_options.chk_pos2_baselen.place(x=20, y=330)


def setting2_2(N: int, variables_options) -> None:
    """Show or hide widgets for the Setting2.2 tab."""
    if N == 0:
        variables_options.label_pos2_siggps.place_forget()
        variables_options.label_pos2_sigqzs.place_forget()
        variables_options.label_pos2_siggal.place_forget()
        variables_options.label_pos2_sigbds2.place_forget()
        variables_options.label_pos2_sigbds3.place_forget()
        variables_options.combo_pos2_siggps.place_forget()
        variables_options.combo_pos2_sigqzs.place_forget()
        variables_options.combo_pos2_siggal.place_forget()
        variables_options.combo_pos2_sigbds2.place_forget()
        variables_options.combo_pos2_sigbds3.place_forget()
        variables_options.label_pos2_bds2prn.place_forget()
        variables_options.entry_pos2_bds2prn.place_forget()
        variables_options.label_pos2_bds3igso.place_forget()
        variables_options.entry_pos2_bds3igso.place_forget()
    else:
        variables_options.label_pos2_siggps.place(x=20, y=40)
        variables_options.label_pos2_sigqzs.place(x=20, y=70)
        variables_options.label_pos2_siggal.place(x=20, y=100)
        variables_options.label_pos2_sigbds2.place(x=20, y=130)
        variables_options.label_pos2_sigbds3.place(x=20, y=160)
        variables_options.combo_pos2_siggps.place(x=340, y=40)
        variables_options.combo_pos2_sigqzs.place(x=340, y=70)
        variables_options.combo_pos2_siggal.place(x=340, y=100)
        variables_options.combo_pos2_sigbds2.place(x=340, y=130)
        variables_options.combo_pos2_sigbds3.place(x=340, y=160)
        variables_options.label_pos2_bds2prn.place(x=20, y=190)
        variables_options.entry_pos2_bds2prn.place(
            x=340, y=190, width=variables_options.combo_pos2_sigbds3.winfo_reqwidth()
        )
        variables_options.label_pos2_bds3igso.place(x=20, y=220)
        variables_options.entry_pos2_bds3igso.place(
            x=340, y=220, width=variables_options.combo_pos2_sigbds3.winfo_reqwidth()
        )


def output(N: int, variables_options) -> None:
    """Show or hide widgets for the Output tab."""
    if N == 0:
        variables_options.label_out_solformat.place_forget()
        variables_options.label_out_outhead_outopt.place_forget()
        variables_options.label_out_timesys_timeform_timendec.place_forget()
        variables_options.label_out_degform.place_forget()
        variables_options.label_out_fieldsep.place_forget()
        variables_options.label_out_height.place_forget()
        variables_options.label_out_geoid.place_forget()
        variables_options.label_out_solstatic.place_forget()
        variables_options.label_out_nmeaintv.place_forget()
        variables_options.label_out_outstat.place_forget()
        variables_options.combo_outsolformat.place_forget()
        variables_options.combo_out_outhead_outopt1.place_forget()
        variables_options.combo_out_outhead_outopt2.place_forget()
        variables_options.combo_out_outvel.place_forget()
        variables_options.combo_out_timesys_timeform_timendec1.place_forget()
        variables_options.entry_out_timesys_timeform_timendec2.place_forget()
        variables_options.combo_out_degform.place_forget()
        variables_options.entry_out_fieldsep.place_forget()
        variables_options.combo_out_height.place_forget()
        variables_options.combo_out_geoid.place_forget()
        variables_options.combo_out_solstatic.place_forget()
        variables_options.entry_out_nmeaintv1.place_forget()
        variables_options.entry_out_nmeaintv2.place_forget()
        variables_options.combo_out_outstat.place_forget()
    else:
        variables_options.label_out_solformat.place(x=20, y=40)
        variables_options.label_out_outhead_outopt.place(x=20, y=70)
        variables_options.label_out_timesys_timeform_timendec.place(x=20, y=100)
        variables_options.label_out_degform.place(x=20, y=130)
        variables_options.label_out_fieldsep.place(x=20, y=160)
        variables_options.label_out_height.place(x=20, y=190)
        variables_options.label_out_geoid.place(x=20, y=220)
        variables_options.label_out_solstatic.place(x=20, y=250)
        variables_options.label_out_nmeaintv.place(x=20, y=280)
        variables_options.label_out_outstat.place(x=20, y=310)
        variables_options.combo_outsolformat.place(x=340, y=40)
        variables_options.combo_out_outhead_outopt1.place(x=340, y=70)
        variables_options.combo_out_outhead_outopt2.place(x=430, y=70)
        variables_options.combo_out_outvel.place(x=520, y=70)
        variables_options.combo_out_timesys_timeform_timendec1.place(x=340, y=100)
        variables_options.entry_out_timesys_timeform_timendec2.place(x=520, y=100)
        variables_options.combo_out_degform.place(x=340, y=130)
        variables_options.entry_out_fieldsep.place(x=340, y=160)
        variables_options.combo_out_height.place(x=340, y=190)
        variables_options.combo_out_geoid.place(x=340, y=220)
        variables_options.combo_out_solstatic.place(x=340, y=250)
        variables_options.entry_out_nmeaintv1.place(x=340, y=280)
        variables_options.entry_out_nmeaintv2.place(x=477, y=280)
        variables_options.combo_out_outstat.place(x=340, y=310)


def stats(N: int, variables_options) -> None:
    """Show or hide widgets for the Stats tab."""
    if N == 0:
        variables_options.frame_stat_1.place_forget()
        variables_options.frame_stat_2.place_forget()
        variables_options.label_stat_eratio.place_forget()
        variables_options.label_stat_prnaccelh_prnaccelv.place_forget()
        variables_options.label_stat_errphase.place_forget()
        variables_options.label_stat_errphaseb1.place_forget()
        variables_options.label_stat_errdoppler.place_forget()
        variables_options.label_stat_uraratio.place_forget()
        variables_options.label_stat_prnbias.place_forget()
        variables_options.label_stat_prniono.place_forget()
        variables_options.label_stat_prntrop.place_forget()
        variables_options.label_stat_prnifb.place_forget()
        variables_options.label_stat_prnpos.place_forget()
        variables_options.label_stat_clktab.place_forget()
        variables_options.entry_stat_eratio1.place_forget()
        variables_options.entry_stat_eratio2.place_forget()
        variables_options.entry_stat_errphase1.place_forget()
        variables_options.entry_stat_errphase2.place_forget()
        variables_options.entry_stat_errphaseb1.place_forget()
        variables_options.entry_stat_errdoppler.place_forget()
        variables_options.entry_stat_uraratio.place_forget()
        variables_options.entry_stat_prnaccelh_prnaccelv1.place_forget()
        variables_options.entry_stat_prnaccelh_prnaccelv2.place_forget()
        variables_options.entry_stat_prnbias.place_forget()
        variables_options.entry_stat_prniono.place_forget()
        variables_options.entry_stat_prntrop.place_forget()
        variables_options.entry_stat_prnifb.place_forget()
        variables_options.entry_stat_prnpos.place_forget()
        variables_options.entry_stat_clktab.place_forget()
    else:
        variables_options.frame_stat_1.place(x=10, y=25)
        variables_options.frame_stat_2.place(x=10, y=170)
        variables_options.label_stat_eratio.place(x=20, y=45)
        variables_options.label_stat_errphase.place(x=20, y=70)
        variables_options.label_stat_errphaseb1.place(x=20, y=95)
        variables_options.label_stat_errdoppler.place(x=20, y=120)
        variables_options.label_stat_uraratio.place(x=20, y=145)
        variables_options.label_stat_prnaccelh_prnaccelv.place(x=20, y=185)
        variables_options.label_stat_prnbias.place(x=20, y=210)
        variables_options.label_stat_prniono.place(x=20, y=235)
        variables_options.label_stat_prntrop.place(x=20, y=260)
        variables_options.label_stat_prnifb.place(x=20, y=285)
        variables_options.label_stat_prnpos.place(x=20, y=310)
        variables_options.label_stat_clktab.place(x=20, y=338)
        variables_options.entry_stat_eratio1.place(x=340, y=45)
        variables_options.entry_stat_eratio2.place(x=477, y=45)
        variables_options.entry_stat_errphase1.place(x=340, y=70)
        variables_options.entry_stat_errphase2.place(x=477, y=70)
        variables_options.entry_stat_errphaseb1.place(x=340, y=95)
        variables_options.entry_stat_errdoppler.place(x=340, y=120)
        variables_options.entry_stat_uraratio.place(x=340, y=145)
        variables_options.entry_stat_prnaccelh_prnaccelv1.place(x=340, y=185)
        variables_options.entry_stat_prnaccelh_prnaccelv2.place(x=477, y=185)
        variables_options.entry_stat_prnbias.place(x=340, y=210)
        variables_options.entry_stat_prniono.place(x=340, y=235)
        variables_options.entry_stat_prntrop.place(x=340, y=260)
        variables_options.entry_stat_prnifb.place(x=340, y=285)
        variables_options.entry_stat_prnpos.place(x=340, y=310)
        variables_options.entry_stat_clktab.place(x=340, y=338)


def positions(N: int, variables_options) -> None:
    """Show or hide widgets for the Positions tab."""
    if N == 0:
        variables_options.frame_position_1.place_forget()
        variables_options.frame_position_2.place_forget()
        variables_options.label_position_antde1.place_forget()
        variables_options.label_position_antde2.place_forget()
        variables_options.label_position_file_staposfile.place_forget()
        variables_options.combo_position_postype1.place_forget()
        variables_options.entry_position_pos11.place_forget()
        variables_options.entry_position_pos21.place_forget()
        variables_options.entry_position_pos31.place_forget()
        variables_options.entry_position_anttype1.place_forget()
        variables_options.entry_position_antdele1.place_forget()
        variables_options.entry_position_antdeln1.place_forget()
        variables_options.entry_position_antdelu1.place_forget()
        variables_options.chk_position_anttype1.place_forget()
        variables_options.combo_position_postype2.place_forget()
        variables_options.entry_position_pos12.place_forget()
        variables_options.entry_position_pos22.place_forget()
        variables_options.entry_position_pos32.place_forget()
        variables_options.entry_position_anttype2.place_forget()
        variables_options.entry_position_antdele2.place_forget()
        variables_options.entry_position_antdeln2.place_forget()
        variables_options.entry_position_antdelu2.place_forget()
        variables_options.chk_position_anttype2.place_forget()
        variables_options.btn_position_staposfile.place_forget()
        variables_options.entry_position_staposfile.place_forget()
    else:
        variables_options.frame_position_1.place(x=10, y=25)
        variables_options.frame_position_2.place(x=10, y=160)
        variables_options.label_position_antde1.place(x=380, y=100)
        variables_options.label_position_antde2.place(x=380, y=245)
        variables_options.label_position_file_staposfile.place(x=20, y=305)
        variables_options.combo_position_postype1.place(x=20, y=45)
        variables_options.entry_position_pos11.place(x=20, y=75)
        variables_options.entry_position_pos21.place(x=220, y=75)
        variables_options.entry_position_pos31.place(x=420, y=75)
        variables_options.entry_position_anttype1.place(x=20, y=130)
        variables_options.entry_position_antdele1.place(x=380, y=130)
        variables_options.entry_position_antdeln1.place(x=460, y=130)
        variables_options.entry_position_antdelu1.place(x=540, y=130)
        variables_options.chk_position_anttype1.place(x=20, y=100)
        variables_options.combo_position_postype2.place(x=20, y=190)
        variables_options.entry_position_pos12.place(x=20, y=220)
        variables_options.entry_position_pos22.place(x=220, y=220)
        variables_options.entry_position_pos32.place(x=420, y=220)
        variables_options.entry_position_anttype2.place(x=20, y=275)
        variables_options.entry_position_antdele2.place(x=380, y=275)
        variables_options.entry_position_antdeln2.place(x=460, y=275)
        variables_options.entry_position_antdelu2.place(x=540, y=275)
        variables_options.chk_position_anttype2.place(x=20, y=245)
        variables_options.btn_position_staposfile.place(x=583, y=325, width=20)
        variables_options.entry_position_staposfile.place(x=20, y=328)


def files(N: int, variables_options) -> None:
    """Show or hide widgets for the Files tab."""
    if N == 0:
        variables_options.label_file_satantfile.place_forget()
        variables_options.label_file_geoidfile.place_forget()
        variables_options.label_file_dcbfile.place_forget()
        variables_options.label_file_eopfile.place_forget()
        variables_options.label_file_blqfile.place_forget()
        variables_options.label_file_ionofile.place_forget()
        variables_options.entry_file_satantfile1.place_forget()
        variables_options.entry_file_satantfile2.place_forget()
        variables_options.entry_file_geoidfile.place_forget()
        variables_options.entry_file_dcbfile.place_forget()
        variables_options.entry_file_eopfile.place_forget()
        variables_options.entry_file_blqfile.place_forget()
        variables_options.entry_file_ionofile.place_forget()
        variables_options.btn_file_satantfile1.place_forget()
        variables_options.btn_file_satantfile2.place_forget()
        variables_options.btn_file_geoidfile.place_forget()
        variables_options.btn_file_dcbfile.place_forget()
        variables_options.btn_file_eopfile.place_forget()
        variables_options.btn_file_blqfile.place_forget()
        variables_options.btn_file_ionofile.place_forget()
    else:
        variables_options.label_file_satantfile.place(x=20, y=35)
        variables_options.label_file_geoidfile.place(x=20, y=110)
        variables_options.label_file_dcbfile.place(x=20, y=160)
        variables_options.label_file_eopfile.place(x=20, y=210)
        variables_options.label_file_blqfile.place(x=20, y=260)
        variables_options.label_file_ionofile.place(x=20, y=310)
        variables_options.entry_file_satantfile1.place(x=20, y=55)
        variables_options.entry_file_satantfile2.place(x=20, y=85)
        variables_options.entry_file_geoidfile.place(x=20, y=130)
        variables_options.entry_file_dcbfile.place(x=20, y=180)
        variables_options.entry_file_eopfile.place(x=20, y=230)
        variables_options.entry_file_blqfile.place(x=20, y=280)
        variables_options.entry_file_ionofile.place(x=20, y=330)
        variables_options.btn_file_satantfile1.place(x=583, y=51)
        variables_options.btn_file_satantfile2.place(x=583, y=81)
        variables_options.btn_file_geoidfile.place(x=583, y=126)
        variables_options.btn_file_dcbfile.place(x=583, y=176)
        variables_options.btn_file_eopfile.place(x=583, y=226)
        variables_options.btn_file_blqfile.place(x=583, y=276)
        variables_options.btn_file_ionofile.place(x=583, y=326)


def misc(N: int, variables_options) -> None:
    """Show or hide widgets for the Misc tab."""
    if N == 0:
        variables_options.label_misc_timeinterp.place_forget()
        variables_options.label_misc_sbasatsel.place_forget()
        variables_options.label_misc_rnxopt1.place_forget()
        variables_options.label_misc_rnxopt2.place_forget()
        variables_options.label_misc_pppopt.place_forget()
        variables_options.label_misc_rtcopt.place_forget()
        variables_options.combo_misc_timeinterp.place_forget()
        variables_options.entry_misc_sbasatsel.place_forget()
        variables_options.entry_misc_rnxopt1.place_forget()
        variables_options.entry_misc_rnxopt2.place_forget()
        variables_options.entry_misc_pppopt.place_forget()
        variables_options.entry_misc_rtcopt.place_forget()
    else:
        variables_options.label_misc_timeinterp.place(x=20, y=40)
        variables_options.label_misc_sbasatsel.place(x=20, y=80)
        variables_options.label_misc_rnxopt1.place(x=20, y=120)
        variables_options.label_misc_rnxopt2.place(x=20, y=160)
        variables_options.label_misc_pppopt.place(x=20, y=200)
        variables_options.label_misc_rtcopt.place(x=20, y=240)
        variables_options.combo_misc_timeinterp.place(x=350, y=40)
        variables_options.entry_misc_sbasatsel.place(x=350, y=80)
        variables_options.entry_misc_rnxopt1.place(x=220, y=120)
        variables_options.entry_misc_rnxopt2.place(x=220, y=160)
        variables_options.entry_misc_pppopt.place(x=220, y=200)
        variables_options.entry_misc_rtcopt.place(x=220, y=240)
