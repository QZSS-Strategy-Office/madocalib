# Copyright (C) 2026 Cabinet Office, Japan, All rights reserved.
# Copyright (C) 2026 Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
# SPDX-License-Identifier: BSD-2-Clause
"""Options dialog variables and widget container.

This module defines the widget container used by the Options dialog. It keeps
legacy-compatible field names so that the view layer can place/forget widgets
and read/write values without changes.

Notes:
    - This module does not perform layout; positions are managed by the view.
    - `CONF_SCHEMA` stores known keys (and optional defaults) and is reset by
      `reset_conf_schema()` before loading a conf.
    - Basic widgets (Labels/Entries/Comboboxes/Checkbuttons/Frames/Buttons) are
      constructed here so that the view can safely show/hide/place them.

"""

from __future__ import annotations

import logging
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk
from typing import Any, Dict

logger = logging.getLogger(__name__)


@dataclass
class VariablesOptions:
    """Container for widget references used by the Options dialog.

    Notes:
        Also holds the configuration schema (CONF_SCHEMA) and parsed conf lines.

    """

    # The Toplevel window for this options dialog
    parent: tk.Toplevel

    # --- Tabs / shared ---
    tab_key_active: str | None = None
    default_bg_color: str | None = None
    tab_buttons: dict[int, tk.Button] = field(default_factory=dict)

    # --- Schema & parsed conf lines ---
    CONF_SCHEMA: Dict[str, str] = field(default_factory=dict)
    conf_list: list[Any] | None = None

    # --- Setting1 ---
    label_pos1_posmode: tk.Label | None = None
    label_pos1_navsys: tk.Label | None = None
    label_pos1_frequency_soltype: tk.Label | None = None
    label_pos1_elmask_snrmask: tk.Label | None = None
    label_pos1_dynamics_tidecorr: tk.Label | None = None
    label_pos1_ionoopt: tk.Label | None = None
    label_pos1_tropopt: tk.Label | None = None
    label_pos1_sateph: tk.Label | None = None
    label_pos1_exclsats: tk.Label | None = None

    combo_pos1_posmode: ttk.Combobox | None = None
    combo_pos1_frequency_soltype1: ttk.Combobox | None = None
    combo_pos1_frequency_soltype2: ttk.Combobox | None = None
    combo_pos1_elmask_snrmask1: ttk.Combobox | None = None
    combo_pos1_elmask_snrmask2: ttk.Combobox | None = None
    combo_pos1_dynamics_tidecorr1: ttk.Combobox | None = None
    combo_pos1_dynamics_tidecorr2: ttk.Combobox | None = None
    combo_pos1_ionoopt: ttk.Combobox | None = None
    combo_pos1_tropopt: ttk.Combobox | None = None
    combo_pos1_sateph: ttk.Combobox | None = None
    entry_pos1_exclsats: tk.Entry | None = None

    # navsys and posopt checkboxes
    chk_pos1_posopt1: tk.Checkbutton | None = None
    chk_pos1_posopt2: tk.Checkbutton | None = None
    chk_pos1_posopt3: tk.Checkbutton | None = None
    chk_pos1_posopt4: tk.Checkbutton | None = None
    chk_pos1_posopt5: tk.Checkbutton | None = None
    chk_pos1_posopt6: tk.Checkbutton | None = None

    chk_pos1_navsys1: tk.Checkbutton | None = None
    chk_pos1_navsys2: tk.Checkbutton | None = None
    chk_pos1_navsys3: tk.Checkbutton | None = None
    chk_pos1_navsys4: tk.Checkbutton | None = None
    chk_pos1_navsys5: tk.Checkbutton | None = None

    # Backing IntVars
    checkVar1: tk.IntVar | None = None
    checkVar2: tk.IntVar | None = None
    checkVar3: tk.IntVar | None = None
    checkVar4: tk.IntVar | None = None
    checkVar5: tk.IntVar | None = None
    checkVar6: tk.IntVar | None = None

    checkVarnav1: tk.IntVar | None = None
    checkVarnav2: tk.IntVar | None = None
    checkVarnav3: tk.IntVar | None = None
    checkVarnav4: tk.IntVar | None = None
    checkVarnav5: tk.IntVar | None = None

    # --- Setting2.1 ---
    label_pos2_armode: tk.Label | None = None
    label_pos2_arsys: tk.Label | None = None
    label_pos2_ionocorr: tk.Label | None = None
    label_pos2_arthres: tk.Label | None = None
    label_pos2_arth1_2: tk.Label | None = None
    label_pos2_arlockcnt_arelmask: tk.Label | None = None
    label_pos2_arminfix_elmaskhold: tk.Label | None = None
    label_pos2_aroutcnt_slipthres: tk.Label | None = None
    label_pos2_maxage_syncsol: tk.Label | None = None
    label_pos2_rejionno_rejgdop: tk.Label | None = None
    label_pos2_niter: tk.Label | None = None

    combo_pos2_armode: ttk.Combobox | None = None
    combo_pos2_arsys1: ttk.Combobox | None = None  # GPS
    combo_pos2_arsys2: ttk.Combobox | None = None  # GLO
    combo_pos2_arsys3: ttk.Combobox | None = None  # GAL
    combo_pos2_arsys4: ttk.Combobox | None = None  # QZS
    combo_pos2_arsys5: ttk.Combobox | None = None  # BDS

    combo_pos2_ionocorr1: ttk.Combobox | None = None
    entry_pos2_arthres: tk.Entry | None = None
    entry_pos2_arth1_21: tk.Entry | None = None
    entry_pos2_arth1_22: tk.Entry | None = None
    entry_pos2_arlockcnt_arelmask1: tk.Entry | None = None
    entry_pos2_arelmask = None
    entry_pos2_arlockcnt_arelmask2: tk.Entry | None = None
    entry_pos2_arminfix_elmaskhold1: tk.Entry | None = None
    entry_pos2_AR_Filter_Iter: tk.Entry | None = None
    entry_pos2_arminfix_elmaskhold2: tk.Entry | None = None
    entry_pos2_aroutcnt_slipthres1: tk.Entry | None = None
    entry_pos2_maxage_syncsol1: tk.Entry | None = None
    combo_pos2_maxage_syncsol2: ttk.Combobox | None = None
    entry_pos2_aroutcnt_slipthres2: tk.Entry | None = None
    entry_pos2_rejionno_rejgdop2: tk.Entry | None = None
    entry_pos2_rejionno_rejgdop1: tk.Entry | None = None
    entry_pos2_niter: tk.Entry | None = None
    entry_pos2_baselen: tk.Entry | None = None
    entry_pos2_basesig: tk.Entry | None = None
    chk_pos2_baselen: tk.Checkbutton | None = None
    chk_pos2_baselen: tk.IntVar | None = None

    # --- Setting2.2 ---
    label_pos2_siggps: tk.Label | None = None
    label_pos2_sigqzs: tk.Label | None = None
    label_pos2_siggal: tk.Label | None = None
    label_pos2_sigbds2: tk.Label | None = None
    label_pos2_sigbds3: tk.Label | None = None

    combo_pos2_siggps: ttk.Combobox | None = None
    combo_pos2_sigqzs: ttk.Combobox | None = None
    combo_pos2_siggal: ttk.Combobox | None = None
    combo_pos2_sigbds2: ttk.Combobox | None = None
    combo_pos2_sigbds3: ttk.Combobox | None = None

    # BDS-2 satellites in PRN 1-18 (pos2-bds2prn)
    label_pos2_bds2prn: tk.Label | None = None
    entry_pos2_bds2prn: tk.Entry | None = None

    # BDS-3 IGSO satellites (pos2-bds3igso)
    label_pos2_bds3igso: tk.Label | None = None
    entry_pos2_bds3igso: tk.Entry | None = None

    # --- Output ---
    label_out_solformat: tk.Label | None = None
    label_out_outhead_outopt: tk.Label | None = None
    label_out_timesys_timeform_timendec: tk.Label | None = None
    label_out_degform: tk.Label | None = None
    label_out_fieldsep: tk.Label | None = None
    label_out_height: tk.Label | None = None
    label_out_geoid: tk.Label | None = None
    label_out_solstatic: tk.Label | None = None
    label_out_nmeaintv: tk.Label | None = None
    label_out_outstat: tk.Label | None = None

    combo_outsolformat: ttk.Combobox | None = None
    combo_out_outhead_outopt1: ttk.Combobox | None = None
    combo_out_outhead_outopt2: ttk.Combobox | None = None
    combo_out_outvel: ttk.Combobox | None = None
    combo_out_timesys_timeform_timendec1: ttk.Combobox | None = None
    entry_out_timesys_timeform_timendec2: tk.Entry | None = None
    combo_out_degform: ttk.Combobox | None = None
    entry_out_fieldsep: tk.Entry | None = None
    combo_out_height: ttk.Combobox | None = None
    combo_out_geoid: ttk.Combobox | None = None
    combo_out_solstatic: ttk.Combobox | None = None
    entry_out_nmeaintv1: tk.Entry | None = None
    entry_out_nmeaintv2: tk.Entry | None = None
    combo_out_outstat: ttk.Combobox | None = None

    # --- Stats ---
    frame_stat_1: tk.LabelFrame | None = None
    frame_stat_2: tk.LabelFrame | None = None
    label_stat_eratio: tk.Label | None = None
    label_stat_prnaccelh_prnaccelv: tk.Label | None = None
    label_stat_errphase: tk.Label | None = None
    label_stat_errphaseb1: tk.Label | None = None
    label_stat_errdoppler: tk.Label | None = None
    label_stat_uraratio: tk.Label | None = None
    label_stat_prnbias: tk.Label | None = None
    label_stat_prniono: tk.Label | None = None
    label_stat_prntrop: tk.Label | None = None
    label_stat_prnifb: tk.Label | None = None
    label_stat_prnpos: tk.Label | None = None
    label_stat_clktab: tk.Label | None = None

    entry_stat_eratio1: tk.Entry | None = None
    entry_stat_eratio2: tk.Entry | None = None
    entry_stat_errphase1: tk.Entry | None = None
    entry_stat_errphase2: tk.Entry | None = None
    entry_stat_errphaseb1: tk.Entry | None = None
    entry_stat_errdoppler: tk.Entry | None = None
    entry_stat_uraratio: tk.Entry | None = None
    entry_stat_prnaccelh_prnaccelv1: tk.Entry | None = None
    entry_stat_prnaccelh_prnaccelv2: tk.Entry | None = None
    entry_stat_prnbias: tk.Entry | None = None
    entry_stat_prniono: tk.Entry | None = None
    entry_stat_prntrop: tk.Entry | None = None
    entry_stat_prnifb: tk.Entry | None = None
    entry_stat_prnpos: tk.Entry | None = None
    entry_stat_clktab: tk.Entry | None = None

    # --- Positions ---
    frame_position_1: tk.LabelFrame | None = None
    frame_position_2: tk.LabelFrame | None = None
    label_position_antde1: tk.Label | None = None
    label_position_antde2: tk.Label | None = None
    label_position_file_staposfile: tk.Label | None = None

    combo_position_postype1: ttk.Combobox | None = None
    entry_position_pos11: tk.Entry | None = None
    entry_position_pos21: tk.Entry | None = None
    entry_position_pos31: tk.Entry | None = None
    entry_position_anttype1: tk.Entry | None = None
    entry_position_antdele1: tk.Entry | None = None
    entry_position_antdeln1: tk.Entry | None = None
    entry_position_antdelu1: tk.Entry | None = None
    chk_position_anttype1: tk.Checkbutton | None = None
    chk_position_anttype1: tk.IntVar | None = None

    combo_position_postype2: ttk.Combobox | None = None
    entry_position_pos12: tk.Entry | None = None
    entry_position_pos22: tk.Entry | None = None
    entry_position_pos32: tk.Entry | None = None
    entry_position_anttype2: tk.Entry | None = None
    entry_position_antdele2: tk.Entry | None = None
    entry_position_antdeln2: tk.Entry | None = None
    entry_position_antdelu2: tk.Entry | None = None
    chk_position_anttype2: tk.Checkbutton | None = None
    chk_position_anttype2: tk.IntVar | None = None

    btn_position_staposfile: tk.Button | None = None
    entry_position_staposfile: tk.Entry | None = None

    # --- Files ---
    label_file_satantfile: tk.Label | None = None
    label_file_geoidfile: tk.Label | None = None
    label_file_dcbfile: tk.Label | None = None
    label_file_eopfile: tk.Label | None = None
    label_file_blqfile: tk.Label | None = None
    label_file_ionofile: tk.Label | None = None

    entry_file_satantfile1: tk.Entry | None = None
    entry_file_satantfile2: tk.Entry | None = None
    entry_file_geoidfile: tk.Entry | None = None
    entry_file_dcbfile: tk.Entry | None = None
    entry_file_eopfile: tk.Entry | None = None
    entry_file_blqfile: tk.Entry | None = None
    entry_file_ionofile: tk.Entry | None = None

    btn_file_satantfile1: tk.Button | None = None
    btn_file_satantfile2: tk.Button | None = None
    btn_file_geoidfile: tk.Button | None = None
    btn_file_dcbfile: tk.Button | None = None
    btn_file_eopfile: tk.Button | None = None
    btn_file_blqfile: tk.Button | None = None
    btn_file_ionofile: tk.Button | None = None

    # --- Misc ---
    label_misc_timeinterp: tk.Label | None = None
    label_misc_sbasatsel: tk.Label | None = None
    label_misc_rnxopt1: tk.Label | None = None
    label_misc_rnxopt2: tk.Label | None = None
    label_misc_pppopt: tk.Label | None = None
    label_misc_rtcopt: tk.Label | None = None

    combo_misc_timeinterp: ttk.Combobox | None = None
    entry_misc_sbasatsel: tk.Entry | None = None
    entry_misc_rnxopt1: tk.Entry | None = None
    entry_misc_rnxopt2: tk.Entry | None = None
    entry_misc_pppopt: tk.Entry | None = None
    entry_misc_rtcopt: tk.Entry | None = None

    def __post_init__(self) -> None:
        """Build widgets and initialize the schema.

        Notes:
            Creates IntVars/Entries/Comboboxes/Buttons and calls
            :meth:`reset_conf_schema` to prepare known keys.

        """
        # Initialize known keys
        self.reset_conf_schema()

    def reset_conf_schema(self) -> None:
        """Reset known conf keys and their default values."""
        self.CONF_SCHEMA = {
            # --- Setting1 ---
            "pos1-posmode": "",
            "pos1-frequency": "",
            "pos1-soltype": "",
            "pos1-elmask": "",
            "pos1-snrmask_r": "",
            # "pos1-snrmask_b": "",  # default only
            # "pos1-snrmask_L1": "",  # default only
            # "pos1-snrmask_L2": "",  # default only
            # "pos1-snrmask_L5": "",  # default only
            "pos1-dynamics": "",
            "pos1-tidecorr": "",
            "pos1-ionoopt": "",
            "pos1-tropopt": "",
            "pos1-sateph": "",
            "pos1-posopt1": "",
            "pos1-posopt2": "",
            "pos1-posopt3": "",
            "pos1-posopt4": "",
            "pos1-posopt5": "",
            "pos1-posopt6": "",
            "pos1-exclsats": "",
            "pos1-navsys": "",
            # --- Setting2.1 ---
            "pos2-armode": "",
            "pos2-arsys": "",
            "pos2-gloarmode": "",
            "pos2-ionocorr": "",
            "pos2-arthres": "",
            "pos2-arthres1": "",
            "pos2-arthres2": "",
            "pos2-arlockcnt": "",
            "pos2-arelmask": "",
            "pos2-arminfix": "",
            "pos2-armaxiter": "",
            "pos2-elmaskhold": "",
            "pos2-aroutcnt": "",
            "pos2-maxage": "",
            "pos2-syncsol": "",
            "pos2-slipthres": "",
            "pos2-rejionno": "",
            "pos2-rejgdop": "",
            "pos2-niter": "",
            "pos2-baselen": "",
            "pos2-basesig": "",
            # --- Setting2.2 ---
            "pos2-siggps": "",
            "pos2-sigqzs": "",
            "pos2-siggal": "",
            "pos2-sigbds2": "",
            "pos2-sigbds3": "",
            "pos2-bds2prn": "",
            "pos2-bds3igso": "",
            # --- Output ---
            "out-solformat": "",
            "out-outhead": "",
            "out-outopt": "",
            "out-outvel": "",
            "out-timesys": "",
            "out-timeform": "",
            "out-timendec": "",
            "out-degform": "",
            "out-fieldsep": "",
            # "out-outsingle": "",  # default only
            # "out-maxsolstd": "",  # default only
            "out-height": "",
            "out-geoid": "",
            "out-solstatic": "",
            "out-nmeaintv1": "",
            "out-nmeaintv2": "",
            "out-outstat": "",
            # --- Stats ---
            "stats-eratio1": "",
            "stats-eratio2": "",
            "stats-errphase": "",
            "stats-errphaseel": "",
            "stats-errphasebl": "",
            "stats-errdoppler": "",
            "stats-uraratio": "",
            "stats-prnaccelh": "",
            "stats-prnaccelv": "",
            "stats-prnbias": "",
            "stats-prniono": "",
            "stats-prntrop": "",
            "stats-prnpos": "",
            "stats-clkstab": "",
            "stats-prnifb": "",
            # --- Positions ---
            "ant1-postype": "",
            "ant1-pos1": "",
            "ant1-pos2": "",
            "ant1-pos3": "",
            "ant1-anttype": "",
            "ant1-antdele": "",
            "ant1-antdeln": "",
            "ant1-antdelu": "",
            "ant2-postype": "",
            "ant2-pos1": "",
            "ant2-pos2": "",
            "ant2-pos3": "",
            "ant2-anttype": "",
            "ant2-antdele": "",
            "ant2-antdeln": "",
            "ant2-antdelu": "",
            # "ant2-maxaveep": "",  # default only
            # "ant2-initrst": "",  # default only
            # --- Files ---
            "file-satantfile": "",
            "file-rcvantfile": "",
            "file-staposfile": "",
            "file-geoidfile": "",
            "file-ionofile": "",
            "file-dcbfile": "",
            "file-eopfile": "",
            "file-blqfile": "",
            # "file-tempdir": "", # default only
            # "file-geexefile": "", # default only
            # "file-solstatfile": "", # default only
            # "file-tracefile": "", # default only
            # --- Misc ---
            "misc-timeinterp": "",
            "misc-sbasatsel": "",
            "misc-rnxopt1": "",
            "misc-rnxopt2": "",
            "misc-pppopt": "",
            "misc-rtcmopt": "",
        }
