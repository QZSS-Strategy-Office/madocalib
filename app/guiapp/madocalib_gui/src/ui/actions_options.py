# Copyright (C) 2026 Cabinet Office, Japan, All rights reserved.
# Copyright (C) 2026 Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
# SPDX-License-Identifier: BSD-2-Clause
"""Options window actions.

Thin bridge between UI widgets and .conf parser/renderer.
Preserves comments/blank lines while mapping known keys.

"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox
from typing import Dict, List

from core.runtime import get_conf_path, update_conf_file
from ui.eventbus import post
from ui.ui_helpers import set_entry_value

logger = logging.getLogger(__name__)


def on_click_load_conf(option_window, variables_options, conf_path: str) -> str | None:
    """Open a .conf file and load values into the Options UI."""
    try:
        return load_file_conf(option_window, variables_options, conf_path)
    except Exception as e:
        logger.warning(f"Load failed: {e}")
        return None


def on_click_save_conf(option_window, variables_options, conf_path: str) -> str | None:
    """Collect values from the Options UI and save to a .conf file."""
    try:
        saved_path = save_file_conf(option_window, variables_options, conf_path)
        if saved_path:
            post("message", level="info", text="Conf saved!", clear_after=1500)
            try:
                option_window.destroy()
            except Exception:
                pass
            return conf_path or ""
    except Exception as e:
        post("message", level="error", text=f"Save failed: {e}", clear_after=6000)
        return None


def on_click_save_overwrite_conf(option_window, variables_options) -> None:
    """Ask confirmation and overwrite current conf on approval.

    Notes:
        Returns immediately with no side effects if canceled.

    """
    result = messagebox.askokcancel(
        "Confirmation",
        f"Are you sure you want to overwrite {get_conf_path()}?",
        parent=option_window,
    )
    if result:
        on_click_save_conf(option_window, variables_options, get_conf_path())


# The following open-file helpers are used by Options UI:
def on_click_file_satantfile1(parent, entry_file_satantfile1) -> None:
    """Open a PCV/ANTEX file picker and write the selected path to the entry."""
    file = filedialog.askopenfilename(
        parent=parent,
        filetypes=[("PCV files (*.pcv *.atx)", "*.pcv *.atx")],
    )
    if not file:
        return
    set_entry_value(entry_file_satantfile1, file)


def on_click_file_satantfile2(parent, entry_file_satantfile2) -> None:
    """Open a PCV/ANTEX file picker and write the selected path to the entry."""
    file = filedialog.askopenfilename(
        parent=parent,
        filetypes=[("PCV files (*.pcv *.atx)", "*.pcv *.atx")],
    )
    if not file:
        return
    set_entry_value(entry_file_satantfile2, file)


def on_click_file_geoid(parent, entry_file_geoidfile) -> None:
    """Open a geoid data file picker and set the entry value."""
    file = filedialog.askopenfilename(
        parent=parent, filetypes=[("ALL files (*.*)", "*.*")]
    )
    if not file:
        return
    set_entry_value(entry_file_geoidfile, file)


def on_click_file_dcb(parent, entry_file_dcbfile) -> None:
    """Open a DCB data file picker and set the entry value."""
    file = filedialog.askopenfilename(
        parent=parent, filetypes=[("ALL files (*.*)", "*.*")]
    )
    if not file:
        return
    set_entry_value(entry_file_dcbfile, file)


def on_click_file_eop(parent, entry_file_eopfile) -> None:
    """Open an EOP data file picker and set the entry value."""
    file = filedialog.askopenfilename(
        parent=parent, filetypes=[("ALL files (*.*)", "*.*")]
    )
    if not file:
        return
    set_entry_value(entry_file_eopfile, file)


def on_click_file_blq(parent, entry_file_blqfile) -> None:
    """Open an OTL BLQ file picker and set the entry value."""
    file = filedialog.askopenfilename(
        parent=parent, filetypes=[("ALL files (*.*)", "*.*")]
    )
    if not file:
        return
    set_entry_value(entry_file_blqfile, file)


def on_click_file_ionosphere(parent, entry_file_ionofile) -> None:
    """Open an ionosphere data file picker and set the entry value."""
    file = filedialog.askopenfilename(
        parent=parent, filetypes=[("ALL files (*.*)", "*.*")]
    )
    if not file:
        return
    set_entry_value(entry_file_ionofile, file)


def on_click_file_start_position(parent, entry_position_staposfile) -> None:
    """Open a station position (`.pos`) file picker and set the entry value."""
    file = filedialog.askopenfilename(
        parent=parent, filetypes=[("Position File (*.pos)", "*.pos")]
    )
    if not file:
        return
    set_entry_value(entry_position_staposfile, file)


@dataclass
class PairLine:
    """Key-value pair line (optionally with a trailing comment)."""

    kind: str  # "pair"
    key: str
    value: str
    comment: str | None = None


@dataclass
class CommentLine:
    """A full-line comment (starting with '#')."""

    kind: str  # "comment"
    comment: str


@dataclass
class BlankLine:
    """An empty line."""

    kind: str  # "blank"


ParsedLine = PairLine | CommentLine | BlankLine


def _split_comment_aware(line: str) -> tuple[str, str | None]:
    """Split a raw line at the first *unquoted* `#`.

    Args:
        line (str): Raw line as read from the file.

    Returns:
        tuple[str, str | None]: `(content_without_comment, trailing_comment)`.

    """
    in_single = False
    in_double = False
    for i, ch in enumerate(line):
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif ch == "#" and not in_single and not in_double:
            content = line[:i].rstrip()
            comment = line[i + 1 :].lstrip()
            return content, comment
    return line.rstrip(), None


def parse_config_lines(lines: List[str]) -> List[ParsedLine]:
    """Parse .conf lines preserving pairs, comments, and blanks.

    Args:
        lines (List[str]): Lines from a conf file.

    Returns:
        List[ParsedLine]: Parsed representation suitable for rendering.

    """
    result: List[ParsedLine] = []
    for raw in lines:
        if raw.strip() == "":
            result.append(BlankLine(kind="blank"))
            continue

        content, comment = _split_comment_aware(raw)

        # Full-line comment (leading spaces ignored).
        if raw.lstrip().startswith("#"):
            if comment is not None:
                result.append(CommentLine(kind="comment", comment=comment))
            else:
                result.append(
                    CommentLine(kind="comment", comment=raw.lstrip()[1:].lstrip())
                )
            continue

        # Pair line
        if "=" in content:
            key, value = content.split("=", 1)
            key = key.strip()
            value = value.strip()
            result.append(PairLine(kind="pair", key=key, value=value, comment=comment))
        else:
            # Fallback to a comment line if no '=' is present (preserve as-is).
            if comment is not None:
                result.append(CommentLine(kind="comment", comment=comment))
            else:
                result.append(CommentLine(kind="comment", comment=content))
    return result


def parse_config_file(path: str) -> List[ParsedLine]:
    """Read a conf file and return parsed line objects (encoding-tolerant).

    Args:
        path (str): File path to read.

    Returns:
        List[ParsedLine]: Parsed representation of the file.

    """
    encodings = ("utf-8", "utf-8-sig", "cp932")
    for enc in encodings:
        try:
            with open(path, "r", encoding=enc) as f:
                lines = f.read().splitlines()
            return parse_config_lines(lines)
        except UnicodeDecodeError:
            continue
    # Fallback encoding for legacy Windows environments.
    with open(path, "r", encoding="cp932") as f:
        lines = f.read().splitlines()
    return parse_config_lines(lines)


def render_config_lines(
    parsed: List[ParsedLine],
    inner_conf: Dict[str, str],
    key_width: int = 10,
    value_width: int = 10,
) -> List[str]:
    """Render parsed lines, aligning pairs and preserving comments/blanks.

    Args:
        parsed (List[ParsedLine]): Parsed items from the original file.
        inner_conf (Dict[str, str]): Key-value overrides to apply.
        key_width (int): Minimum key width for padding.
        value_width (int): Minimum value width for padding when a comment exists.

    Returns:
        List[str]: Lines ready to be written back to a file.

    """
    out: List[str] = []
    present_keys: dict[str, str] = {}

    for item in parsed:
        if isinstance(item, PairLine):
            if item.key is None:
                continue
            if len(item.key) < key_width:
                key_part = f"{item.key:<{key_width}}"
            else:
                key_part = f"{item.key}"

            # Prefer 'inner_conf' values when provided.
            value = inner_conf.get(item.key, item.value)
            if len(value) < value_width and item.comment is not None:
                value_part = f"{value:<{value_width}}"
            else:
                value_part = f"{value}"
            base = f"{key_part}={value_part}"
            if item.comment is not None:
                out.append(f"{base} # {item.comment}")
            else:
                out.append(base)
            present_keys[item.key] = "1"

        elif isinstance(item, CommentLine):
            if item.comment.strip() == "":
                out.append("#")
            else:
                out.append(f"# {item.comment}")

        elif isinstance(item, BlankLine):
            out.append("")

        else:
            out.append("")

    # Append missing keys from 'inner_conf' (non-empty values only).
    for key, value in inner_conf.items():
        if key in present_keys:
            continue
        if value is None or str(value).strip() == "":
            continue
        key_part = (
            f"{key:<{key_width}}"
            if isinstance(key, str) and len(key) < key_width
            else f"{key}"
        )
        value_part = f"{value}"
        out.append(f"{key_part}={value_part}")
        present_keys[key] = "1"

    return out


def write_config_file(
    path: str,
    parsed: List[ParsedLine],
    inner_conf: Dict[str, str],
    key_width: int = 10,
    value_width: int = 10,
) -> None:
    """Write a conf file by merging parsed lines with UI overrides.

    Args:
        path (str): Destination file path.
        parsed (List[ParsedLine]): Parsed model from the source conf.
        inner_conf (Dict[str, str]): UI-collected overrides.
        key_width (int): Alignment width for keys.
        value_width (int): Alignment width for values.

    """
    lines = render_config_lines(
        parsed, inner_conf, key_width=key_width, value_width=value_width
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def load_file_conf(parent, variables_options, filepath: str) -> str | None:
    """Open a conf file and populate `VariablesOptions` from key-value pairs.

    Args:
        parent: Parent window owning the dialog.
        variables_options: Options UI object storing widgets and parsed lines.
        filepath (str): Suggested path; if invalid, an open dialog is shown.

    Returns:
        str | None: Actual conf path, or `None` if canceled.

    Notes:
        Resets the schema, updates tooltip text, and sets the global conf path.

    """
    logger.info("filepath=%s", filepath)
    filepath = str(filepath)
    if (not filepath) or (not Path(filepath).is_file()):
        conf_path = get_conf_path()
        dname = str(Path(conf_path).parent)
        fname = str(Path(conf_path).name)
        logger.debug("dir: %s, file: %s", dname, fname)
        filepath = filedialog.askopenfilename(
            parent=parent,
            filetypes=[("CONF files", ".conf")],
            initialdir=dname,
            initialfile=fname,
        )
        # Canceled.
        logger.debug("selected conf path=%s", filepath)
        if len(filepath) == 0:
            post("message", level="info", text="Please load the conf file")
            return None

    variables_options.conf_list = parse_config_file(filepath)
    timesys_val, timeform_val = None, None
    if variables_options.conf_list is None:
        post("set_tooltip_opt", text="Please load the conf file")
        return None

    # Reset the schema and tooltip text.
    variables_options.reset_conf_schema()
    # Clear pos2-bds2prn; a conf without the line means "".
    set_entry_value(variables_options.entry_pos2_bds2prn, "")
    # Clear pos2-bds3igso; a conf without the line means "" (C06-C08).
    set_entry_value(variables_options.entry_pos2_bds3igso, "")
    post("set_tooltip_opt", text=filepath)
    update_conf_file(filepath)

    # Apply known keys to the UI.
    for item in variables_options.conf_list:
        if not isinstance(item, PairLine) or item.key is None:
            continue  # skip comments/blanks
        value = (
            item.value
            if item.value is not None
            else variables_options.CONF_SCHEMA.get(item.key)
        )

        # --- Setting1 ---
        match item.key:
            case "pos1-posmode":
                _set_combo_value(
                    variables_options.combo_pos1_posmode, value, variables_options
                )
            case "pos1-frequency":
                _set_combo_value(
                    variables_options.combo_pos1_frequency_soltype1,
                    value,
                    variables_options,
                )
            case "pos1-soltype":
                _set_combo_value(
                    variables_options.combo_pos1_frequency_soltype2,
                    value,
                    variables_options,
                )
            case "pos1-elmask":
                _set_combo_value(
                    variables_options.combo_pos1_elmask_snrmask1,
                    value,
                    variables_options,
                )
            case "pos1-snrmask_r":
                _set_combo_value(
                    variables_options.combo_pos1_elmask_snrmask2,
                    value,
                    variables_options,
                )
            case "pos1-dynamics":
                _set_combo_value(
                    variables_options.combo_pos1_dynamics_tidecorr1,
                    value,
                    variables_options,
                )
            case "pos1-tidecorr":
                _set_combo_value(
                    variables_options.combo_pos1_dynamics_tidecorr2,
                    value,
                    variables_options,
                )
            case "pos1-ionoopt":
                _set_combo_value(
                    variables_options.combo_pos1_ionoopt, value, variables_options
                )
            case "pos1-tropopt":
                _set_combo_value(
                    variables_options.combo_pos1_tropopt, value, variables_options
                )
            case "pos1-sateph":
                _set_combo_value(
                    variables_options.combo_pos1_sateph, value, variables_options
                )
            case "pos1-posopt1":
                _set_checkbutton_state_from_value(
                    variables_options.chk_pos1_posopt1, value
                )
            case "pos1-posopt2":
                _set_checkbutton_state_from_value(
                    variables_options.chk_pos1_posopt2, value
                )
            case "pos1-posopt3":
                _set_checkbutton_state_from_value(
                    variables_options.chk_pos1_posopt3, value
                )
            case "pos1-posopt4":
                _set_checkbutton_state_from_value(
                    variables_options.chk_pos1_posopt4, value
                )
            case "pos1-posopt5":
                _set_checkbutton_state_from_value(
                    variables_options.chk_pos1_posopt5, value
                )
            case "pos1-posopt6":
                _set_checkbutton_state_from_value(
                    variables_options.chk_pos1_posopt6, value
                )
            case "pos1-exclsats":
                set_entry_value(variables_options.entry_pos1_exclsats, value)
            case "pos1-navsys":
                _set_chk_navs(value, variables_options)

            # --- Setting2.1 ---
            case "pos2-armode":
                _set_combo_value(
                    variables_options.combo_pos2_armode, value, variables_options
                )
            case "pos2-arsys":
                _set_combo_gps_arsys(variables_options.combo_pos2_arsys1, value)
                _set_combo_gal_arsys(variables_options.combo_pos2_arsys3, value)
                _set_combo_qzs_arsys(variables_options.combo_pos2_arsys4, value)
                _set_combo_bds_arsys(variables_options.combo_pos2_arsys5, value)
            case "pos2-gloarmode":
                set_entry_value(variables_options.combo_pos2_arsys2, value)
            case "pos2-ionocorr":
                _set_combo_value(
                    variables_options.combo_pos2_ionocorr1, value, variables_options
                )
            case "pos2-arthres":
                set_entry_value(variables_options.entry_pos2_arthres, value)
            case "pos2-arthres1":
                set_entry_value(variables_options.entry_pos2_arth1_21, value)
            case "pos2-arthres2":
                set_entry_value(variables_options.entry_pos2_arth1_22, value)
            case "pos2-arlockcnt":
                set_entry_value(variables_options.entry_pos2_arlockcnt_arelmask1, value)
            case "pos2-arelmask":
                set_entry_value(variables_options.entry_pos2_arlockcnt_arelmask2, value)
            case "pos2-arminfix":
                set_entry_value(
                    variables_options.entry_pos2_arminfix_elmaskhold1, value
                )
            case "pos2-armaxiter":
                set_entry_value(variables_options.entry_pos2_AR_Filter_Iter, value)
            case "pos2-elmaskhold":
                set_entry_value(
                    variables_options.entry_pos2_arminfix_elmaskhold2, value
                )
            case "pos2-aroutcnt":
                set_entry_value(variables_options.entry_pos2_aroutcnt_slipthres1, value)
            case "pos2-maxage":
                set_entry_value(variables_options.entry_pos2_maxage_syncsol1, value)
            case "pos2-syncsol":
                _set_combo_value(
                    variables_options.combo_pos2_maxage_syncsol2,
                    value,
                    variables_options,
                )
            case "pos2-slipthres":
                set_entry_value(variables_options.entry_pos2_aroutcnt_slipthres2, value)
            case "pos2-rejionno":
                set_entry_value(variables_options.entry_pos2_rejionno_rejgdop2, value)
            case "pos2-rejgdop":
                set_entry_value(variables_options.entry_pos2_rejionno_rejgdop1, value)
            case "pos2-niter":
                set_entry_value(variables_options.entry_pos2_niter, value)
            case "pos2-baselen":
                set_entry_value(variables_options.entry_pos2_baselen, value)
                _set_checkbutton_state_from_value(
                    variables_options.chk_pos2_baselen, value
                )
            case "pos2-basesig":
                set_entry_value(variables_options.entry_pos2_basesig, value)

            # --- Setting2.2 ---
            case "pos2-siggps":
                set_entry_value(variables_options.combo_pos2_siggps, value)
            case "pos2-sigqzs":
                set_entry_value(variables_options.combo_pos2_sigqzs, value)
            case "pos2-siggal":
                set_entry_value(variables_options.combo_pos2_siggal, value)
            case "pos2-sigbds2":
                set_entry_value(variables_options.combo_pos2_sigbds2, value)
            case "pos2-sigbds3":
                set_entry_value(variables_options.combo_pos2_sigbds3, value)
            case "pos2-bds2prn":
                set_entry_value(variables_options.entry_pos2_bds2prn, value)
            case "pos2-bds3igso":
                set_entry_value(variables_options.entry_pos2_bds3igso, value)

            # --- Output ---
            case "out-solformat":
                _set_combo_value(
                    variables_options.combo_outsolformat, value, variables_options
                )
            case "out-outhead":
                _set_combo_value(
                    variables_options.combo_out_outhead_outopt1,
                    value,
                    variables_options,
                )
            case "out-outopt":
                _set_combo_value(
                    variables_options.combo_out_outhead_outopt2,
                    value,
                    variables_options,
                )
            case "out-outvel":
                _set_combo_value(
                    variables_options.combo_out_outvel, value, variables_options
                )
            case "out-timesys":
                timesys_val = value
            case "out-timeform":
                timeform_val = value
            case "out-timendec":
                set_entry_value(
                    variables_options.entry_out_timesys_timeform_timendec2, value
                )
            case "out-degform":
                _set_combo_value(
                    variables_options.combo_out_degform, value, variables_options
                )
            case "out-fieldsep":
                set_entry_value(variables_options.entry_out_fieldsep, value)
            case "out-height":
                _set_combo_value(
                    variables_options.combo_out_height, value, variables_options
                )
            case "out-geoid":
                _set_combo_value(
                    variables_options.combo_out_geoid, value, variables_options
                )
            case "out-solstatic":
                _set_combo_value(
                    variables_options.combo_out_solstatic, value, variables_options
                )
            case "out-nmeaintv1":
                set_entry_value(variables_options.entry_out_nmeaintv1, value)
            case "out-nmeaintv2":
                set_entry_value(variables_options.entry_out_nmeaintv2, value)
            case "out-outstat":
                _set_combo_value(
                    variables_options.combo_out_outstat, value, variables_options
                )

            # --- Stats ---
            case "stats-eratio1":
                set_entry_value(variables_options.entry_stat_eratio1, value)
            case "stats-eratio2":
                set_entry_value(variables_options.entry_stat_eratio2, value)
            case "stats-errphase":
                set_entry_value(variables_options.entry_stat_errphase1, value)
            case "stats-errphaseel":
                set_entry_value(variables_options.entry_stat_errphase2, value)
            case "stats-errphasebl":
                set_entry_value(variables_options.entry_stat_errphaseb1, value)
            case "stats-errdoppler":
                set_entry_value(variables_options.entry_stat_errdoppler, value)
            case "stats-uraratio":
                set_entry_value(variables_options.entry_stat_uraratio, value)
            case "stats-prnaccelh":
                set_entry_value(
                    variables_options.entry_stat_prnaccelh_prnaccelv1, value
                )
            case "stats-prnaccelv":
                set_entry_value(
                    variables_options.entry_stat_prnaccelh_prnaccelv2, value
                )
            case "stats-prnbias":
                set_entry_value(variables_options.entry_stat_prnbias, value)
            case "stats-prniono":
                set_entry_value(variables_options.entry_stat_prniono, value)
            case "stats-prntrop":
                set_entry_value(variables_options.entry_stat_prntrop, value)
            case "stats-prnpos":
                set_entry_value(variables_options.entry_stat_prnpos, value)
            case "stats-clkstab":
                set_entry_value(variables_options.entry_stat_clktab, value)
            case "stats-prnifb":
                set_entry_value(variables_options.entry_stat_prnifb, value)

            # --- Positions ---
            case "ant1-postype":
                variables_options.combo_position_postype1.set(value)
            case "ant1-pos1":
                set_entry_value(variables_options.entry_position_pos11, value)
            case "ant1-pos2":
                set_entry_value(variables_options.entry_position_pos21, value)
            case "ant1-pos3":
                set_entry_value(variables_options.entry_position_pos31, value)
            case "ant1-anttype":
                set_entry_value(variables_options.entry_position_anttype1, value)
                variables_options.chk_position_anttype1.deselect()
                if value and len(value) > 0 and value[0] == "*":
                    variables_options.chk_position_anttype1.select()
            case "ant1-antdele":
                set_entry_value(variables_options.entry_position_antdele1, value)
            case "ant1-antdeln":
                set_entry_value(variables_options.entry_position_antdeln1, value)
            case "ant1-antdelu":
                set_entry_value(variables_options.entry_position_antdelu1, value)

            case "ant2-postype":
                variables_options.combo_position_postype2.set(value)
            case "ant2-pos1":
                set_entry_value(variables_options.entry_position_pos12, value)
            case "ant2-pos2":
                set_entry_value(variables_options.entry_position_pos22, value)
            case "ant2-pos3":
                set_entry_value(variables_options.entry_position_pos32, value)
            case "ant2-anttype":
                set_entry_value(variables_options.entry_position_anttype2, value)
                if value[:1] == "*":
                    variables_options.chk_position_anttype2.select()
                else:
                    variables_options.chk_position_anttype2.deselect()
            case "ant2-antdele":
                set_entry_value(variables_options.entry_position_antdele2, value)
            case "ant2-antdeln":
                set_entry_value(variables_options.entry_position_antdeln2, value)
            case "ant2-antdelu":
                set_entry_value(variables_options.entry_position_antdelu2, value)

            # --- Files ---
            case "file-satantfile":
                set_entry_value(variables_options.entry_file_satantfile1, value)
            case "file-rcvantfile":
                set_entry_value(variables_options.entry_file_satantfile2, value)
            case "file-staposfile":
                set_entry_value(variables_options.entry_position_staposfile, value)
            case "file-geoidfile":
                set_entry_value(variables_options.entry_file_geoidfile, value)
            case "file-ionofile":
                set_entry_value(variables_options.entry_file_ionofile, value)
            case "file-dcbfile":
                set_entry_value(variables_options.entry_file_dcbfile, value)
            case "file-eopfile":
                set_entry_value(variables_options.entry_file_eopfile, value)
            case "file-blqfile":
                set_entry_value(variables_options.entry_file_blqfile, value)

            # --- Misc ---
            case "misc-timeinterp":
                _set_combo_value(
                    variables_options.combo_misc_timeinterp, value, variables_options
                )
            case "misc-sbasatsel":
                set_entry_value(variables_options.entry_misc_sbasatsel, value)
            case "misc-rnxopt1":
                set_entry_value(variables_options.entry_misc_rnxopt1, value)
            case "misc-rnxopt2":
                set_entry_value(variables_options.entry_misc_rnxopt2, value)
            case "misc-pppopt":
                set_entry_value(variables_options.entry_misc_pppopt, value)
            case "misc-rtcmopt":
                set_entry_value(variables_options.entry_misc_rtcopt, value)

            case _:
                # Unknown keys are kept in conf_list and preserved on save.
                pass
    # Apply combined time format after scanning all keys
    if timesys_val or timeform_val:
        disp = _compute_timeformat_display(timesys_val, timeform_val)
        if disp:
            variables_options.combo_out_timesys_timeform_timendec1.set(disp)

    return filepath


def save_file_conf(option_window, variables_options, filepath: str) -> str | None:
    """Collect values from `VariablesOptions` and write a conf file.

    Args:
        option_window: Parent window for the save dialog.
        variables_options: Options UI instance providing widget values and parsed lines.
        filepath (str): If empty, shows "Save As"; otherwise overwrites path.

    Notes:
        Preserves comments via merging with the original parsed list.

    """
    logger.debug("filepath=%s", filepath)

    # Build a map(key -> value) from the UI widgets
    out: Dict[str, str] = {}

    for key, _ in variables_options.CONF_SCHEMA.items():
        val = ""
        match key:
            # --- Setting1 ---
            case "pos1-posmode":
                val = variables_options.combo_pos1_posmode.get()
            case "pos1-frequency":
                val = variables_options.combo_pos1_frequency_soltype1.get()
            case "pos1-soltype":
                val = variables_options.combo_pos1_frequency_soltype2.get()
            case "pos1-elmask":
                val = variables_options.combo_pos1_elmask_snrmask1.get()
            case "pos1-snrmask_r":
                val = variables_options.combo_pos1_elmask_snrmask2.get()
            case "pos1-dynamics":
                val = variables_options.combo_pos1_dynamics_tidecorr1.get()
            case "pos1-tidecorr":
                val = variables_options.combo_pos1_dynamics_tidecorr2.get()
            case "pos1-ionoopt":
                val = variables_options.combo_pos1_ionoopt.get()
            case "pos1-tropopt":
                val = variables_options.combo_pos1_tropopt.get()
            case "pos1-sateph":
                val = variables_options.combo_pos1_sateph.get()
            case "pos1-posopt1":
                val = _bool_to_onoff(variables_options.checkVar1)
            case "pos1-posopt2":
                val = _bool_to_onoff(variables_options.checkVar2)
            case "pos1-posopt3":
                val = _bool_to_onoff(variables_options.checkVar3)
            case "pos1-posopt4":
                val = _bool_to_onoff(variables_options.checkVar4)
            case "pos1-posopt5":
                val = _bool_to_onoff(variables_options.checkVar5)
            case "pos1-posopt6":
                val = _bool_to_onoff(variables_options.checkVar6)
            case "pos1-exclsats":
                val = variables_options.entry_pos1_exclsats.get()
            case "pos1-navsys":
                val = _compute_navsys_mask(variables_options)

            # --- Setting2.1 ---
            case "pos2-armode":
                val = variables_options.combo_pos2_armode.get()
            case "pos2-arsys":
                val = _compute_arsys_mask(variables_options)
            case "pos2-gloarmode":
                val = variables_options.combo_pos2_arsys2.get()
            case "pos2-ionocorr":
                val = variables_options.combo_pos2_ionocorr1.get()
            case "pos2-arthres":
                val = variables_options.entry_pos2_arthres.get()
            case "pos2-arthres1":
                val = variables_options.entry_pos2_arth1_21.get()
            case "pos2-arthres2":
                val = variables_options.entry_pos2_arth1_22.get()
            case "pos2-arlockcnt":
                val = variables_options.entry_pos2_arlockcnt_arelmask1.get()
            case "pos2-arelmask":
                val = variables_options.entry_pos2_arlockcnt_arelmask2.get()
            case "pos2-arminfix":
                val = variables_options.entry_pos2_arminfix_elmaskhold1.get()
            case "pos2-armaxiter":
                val = variables_options.entry_pos2_AR_Filter_Iter.get()
            case "pos2-elmaskhold":
                val = variables_options.entry_pos2_arminfix_elmaskhold2.get()
            case "pos2-aroutcnt":
                val = variables_options.entry_pos2_aroutcnt_slipthres1.get()
            case "pos2-maxage":
                val = variables_options.entry_pos2_maxage_syncsol1.get()
            case "pos2-syncsol":
                val = variables_options.combo_pos2_maxage_syncsol2.get()
            case "pos2-slipthres":
                val = variables_options.entry_pos2_aroutcnt_slipthres2.get()
            case "pos2-rejionno":
                val = variables_options.entry_pos2_rejionno_rejgdop2.get()
            case "pos2-rejgdop":
                val = variables_options.entry_pos2_rejionno_rejgdop1.get()
            case "pos2-niter":
                val = variables_options.entry_pos2_niter.get()
            case "pos2-baselen":
                val = variables_options.entry_pos2_baselen.get()
            case "pos2-basesig":
                val = variables_options.entry_pos2_basesig.get()

            # --- Setting2.2 ---
            case "pos2-siggps":
                val = variables_options.combo_pos2_siggps.get()
            case "pos2-sigqzs":
                val = variables_options.combo_pos2_sigqzs.get()
            case "pos2-siggal":
                val = variables_options.combo_pos2_siggal.get()
            case "pos2-sigbds2":
                val = variables_options.combo_pos2_sigbds2.get()
            case "pos2-sigbds3":
                val = variables_options.combo_pos2_sigbds3.get()
            case "pos2-bds2prn":
                val = variables_options.entry_pos2_bds2prn.get()
            case "pos2-bds3igso":
                val = variables_options.entry_pos2_bds3igso.get()

            # --- Output ---
            case "out-solformat":
                val = _normalize_output_format(
                    variables_options.combo_outsolformat.get()
                )
            case "out-outhead":
                val = variables_options.combo_out_outhead_outopt1.get()
            case "out-outopt":
                val = variables_options.combo_out_outhead_outopt2.get()
            case "out-outvel":
                val = variables_options.combo_out_outvel.get()
            case "out-timesys":
                val = _normalize_output_format(
                    variables_options.combo_out_timesys_timeform_timendec1.get(), a=0
                )
            case "out-timeform":
                val = _normalize_output_format(
                    variables_options.combo_out_timesys_timeform_timendec1.get(), a=1
                )
            case "out-timendec":
                val = variables_options.entry_out_timesys_timeform_timendec2.get()
            case "out-degform":
                val = _normalize_output_format(
                    variables_options.combo_out_degform.get()
                )
            case "out-fieldsep":
                val = variables_options.entry_out_fieldsep.get()
            case "out-height":
                val = variables_options.combo_out_height.get()
            case "out-geoid":
                val = variables_options.combo_out_geoid.get()
            case "out-solstatic":
                val = variables_options.combo_out_solstatic.get()
            case "out-nmeaintv1":
                val = variables_options.entry_out_nmeaintv1.get()
            case "out-nmeaintv2":
                val = variables_options.entry_out_nmeaintv2.get()
            case "out-outstat":
                val = _normalize_output_format(
                    variables_options.combo_out_outstat.get()
                )

            # --- Stats ---
            case "stats-eratio1":
                val = variables_options.entry_stat_eratio1.get()
            case "stats-eratio2":
                val = variables_options.entry_stat_eratio2.get()
            case "stats-errphase":
                val = variables_options.entry_stat_errphase1.get()
            case "stats-errphaseel":
                val = variables_options.entry_stat_errphase2.get()
            case "stats-errphasebl":
                val = variables_options.entry_stat_errphaseb1.get()
            case "stats-errdoppler":
                val = variables_options.entry_stat_errdoppler.get()
            case "stats-uraratio":
                val = variables_options.entry_stat_uraratio.get()
            case "stats-prnaccelh":
                val = variables_options.entry_stat_prnaccelh_prnaccelv1.get()
            case "stats-prnaccelv":
                val = variables_options.entry_stat_prnaccelh_prnaccelv2.get()
            case "stats-prnbias":
                val = variables_options.entry_stat_prnbias.get()
            case "stats-prniono":
                val = variables_options.entry_stat_prniono.get()
            case "stats-prntrop":
                val = variables_options.entry_stat_prntrop.get()
            case "stats-prnpos":
                val = variables_options.entry_stat_prnpos.get()
            case "stats-clkstab":
                val = variables_options.entry_stat_clktab.get()
            case "stats-prnifb":
                val = variables_options.entry_stat_prnifb.get()

            # --- Positions ---
            case "ant1-postype":
                val = variables_options.combo_position_postype1.get()
            case "ant1-pos1":
                val = variables_options.entry_position_pos11.get()
            case "ant1-pos2":
                val = variables_options.entry_position_pos21.get()
            case "ant1-pos3":
                val = variables_options.entry_position_pos31.get()
            case "ant1-anttype":
                val = variables_options.entry_position_anttype1.get()
            case "ant1-antdele":
                val = variables_options.entry_position_antdele1.get()
            case "ant1-antdeln":
                val = variables_options.entry_position_antdeln1.get()
            case "ant1-antdelu":
                val = variables_options.entry_position_antdelu1.get()

            case "ant2-postype":
                val = variables_options.combo_position_postype2.get()
            case "ant2-pos1":
                val = variables_options.entry_position_pos12.get()
            case "ant2-pos2":
                val = variables_options.entry_position_pos22.get()
            case "ant2-pos3":
                val = variables_options.entry_position_pos32.get()
            case "ant2-anttype":
                val = variables_options.entry_position_anttype2.get()
            case "ant2-antdele":
                val = variables_options.entry_position_antdele2.get()
            case "ant2-antdeln":
                val = variables_options.entry_position_antdeln2.get()
            case "ant2-antdelu":
                val = variables_options.entry_position_antdelu2.get()

            # --- Files ---
            case "file-satantfile":
                val = variables_options.entry_file_satantfile1.get()
            case "file-rcvantfile":
                val = variables_options.entry_file_satantfile2.get()
            case "file-staposfile":
                val = variables_options.entry_position_staposfile.get()
            case "file-geoidfile":
                val = variables_options.entry_file_geoidfile.get()
            case "file-ionofile":
                val = variables_options.entry_file_ionofile.get()
            case "file-dcbfile":
                val = variables_options.entry_file_dcbfile.get()
            case "file-eopfile":
                val = variables_options.entry_file_eopfile.get()
            case "file-blqfile":
                val = variables_options.entry_file_blqfile.get()

            case "misc-timeinterp":
                val = variables_options.combo_misc_timeinterp.get()
            case "misc-sbasatsel":
                val = variables_options.entry_misc_sbasatsel.get()
            case "misc-rnxopt1":
                val = variables_options.entry_misc_rnxopt1.get()
            case "misc-rnxopt2":
                val = variables_options.entry_misc_rnxopt2.get()
            case "misc-pppopt":
                val = variables_options.entry_misc_pppopt.get()
            case "misc-rtcmopt":
                val = variables_options.entry_misc_rtcopt.get()

            case _:
                # Unknown keys: keep the default (schema) or empty.
                val = variables_options.CONF_SCHEMA.get(key, "")

        if val is None:
            val = ""
        out[key] = str(val)

    # Decide file path (Save As if empty)
    actual_path = filepath
    if not actual_path:
        actual_path = filedialog.asksaveasfilename(
            parent=option_window,
            defaultextension=".conf",
            filetypes=[("CONF files", ".conf")],
        )
        if not actual_path:
            return None

    # Render by merging with original parsed list (preserving comments)
    parsed = variables_options.conf_list or []
    write_config_file(actual_path, parsed, out, key_width=10, value_width=10)

    # Update tooltip & current conf path
    update_conf_file(actual_path)
    post("set_tooltip_opt", text=actual_path)

    return actual_path


def _set_combo_value(combo, value, variables_options) -> None:
    """Map internal codes."""
    if value[:3] == "llh":
        combo.set("Lat/Lon/Height")
    elif value[:3] == "xyz":
        combo.set("X/Y/Z-ECEF")
    elif value[:3] == "enu":
        combo.set("E/N/U-Baseline")
    elif value[:4] == "nmea":
        combo.set("NMEA 0183")
    elif value[:1] == "0" and combo == variables_options.combo_out_outstat:
        combo.set("off")
    elif value[:1] == "1" and combo == variables_options.combo_out_outstat:
        combo.set("state")
    elif value[:1] == "2" and combo == variables_options.combo_out_outstat:
        combo.set("residual")
    elif value[:3] == "deg":
        combo.set("ddd.ddddddd")
    elif value[:3] == "dms":
        combo.set("ddd mm ss.sss")
    else:
        combo.set(value)


def _set_combo_gps_arsys(x, value) -> None:
    """Map AR system mask to 'on'/'off' for GPS."""
    try:
        iv = int(value)
    except (ValueError, TypeError):
        logger.warning("invalid value: %r", value)
        return
    x.set("on" if (iv & 1) == 1 else "off")


def _set_combo_glo_arsys(x, value) -> None:
    """Map AR system mask to 'on'/'off' for GLONASS."""
    try:
        iv = int(value)
    except (ValueError, TypeError):
        logger.warning("invalid value: %r", value)
        return
    x.set("on" if (iv & 4) == 4 else "off")


def _set_combo_gal_arsys(x, value) -> None:
    """Map AR system mask to 'on'/'off' for Galileo."""
    try:
        iv = int(value)
    except (ValueError, TypeError):
        logger.warning("invalid value: %r", value)
        return
    x.set("on" if (iv & 8) == 8 else "off")


def _set_combo_qzs_arsys(x, value) -> None:
    """Map AR system mask to 'on'/'off' for QZSS."""
    try:
        iv = int(value)
    except (ValueError, TypeError):
        logger.warning("invalid value: %r", value)
        return
    x.set("on" if (iv & 16) == 16 else "off")


def _set_combo_bds_arsys(x, value) -> None:
    """Map AR system mask to 'on'/'off' for BeiDou."""
    try:
        iv = int(value)
    except (ValueError, TypeError):
        logger.warning("invalid value: %r", value)
        return
    x.set("on" if (iv & 32) == 32 else "off")


def _set_checkbutton_state_from_value(x, value) -> None:
    """Set Checkbutton state from truthy string (on/1) or clear otherwise."""
    if not value:
        x.deselect()
    elif value[:2] == "on" or value[0] == "1":
        x.select()
    else:
        x.deselect()


def _set_chk_navs(value, variables_options) -> None:
    """Decode navsys bit mask and set satellite-system checkbuttons."""
    variables_options.chk_pos1_navsys1.deselect()
    variables_options.chk_pos1_navsys2.deselect()
    variables_options.chk_pos1_navsys3.deselect()
    variables_options.chk_pos1_navsys4.deselect()
    variables_options.chk_pos1_navsys5.deselect()
    try:
        iv = int(value)
    except (ValueError, TypeError):
        logger.warning("invalid value: %r", value)
        return
    if iv & 1 == 1:
        variables_options.chk_pos1_navsys1.select()
    if iv & 4 == 4:
        variables_options.chk_pos1_navsys2.select()
    if iv & 8 == 8:
        variables_options.chk_pos1_navsys3.select()
    if iv & 16 == 16:
        variables_options.chk_pos1_navsys4.select()
    if iv & 32 == 32:
        variables_options.chk_pos1_navsys5.select()


def _bool_to_onoff(name) -> str:
    """Return 'on' for IntVar==1, else 'off'."""
    return "on" if name.get() == 1 else "off"


def _compute_navsys_mask(variables_options) -> str:
    """Compute navsys bit mask from the five satellite-system checkbuttons."""
    sys_nav = 0
    if variables_options.checkVarnav1.get() == 1:
        sys_nav += 1
    if variables_options.checkVarnav2.get() == 1:
        sys_nav += 4
    if variables_options.checkVarnav3.get() == 1:
        sys_nav += 8
    if variables_options.checkVarnav4.get() == 1:
        sys_nav += 16
    if variables_options.checkVarnav5.get() == 1:
        sys_nav += 32
    return str(sys_nav)


def _compute_arsys_mask(variables_options) -> str:
    """Compute AR system bit mask from per-constellation on/off Comboboxes."""
    sys_ar = 0
    if variables_options.combo_pos2_arsys1.get() == "on":
        sys_ar += 1
    if variables_options.combo_pos2_arsys3.get() == "on":
        sys_ar += 8
    if variables_options.combo_pos2_arsys4.get() == "on":
        sys_ar += 16
    if variables_options.combo_pos2_arsys5.get() == "on":
        sys_ar += 32
    return str(sys_ar)


def _normalize_output_format(value, a=0) -> str:
    """Normalize display text to internal code used by rnx2rtkp.

    Args:
        value: Display string from the combobox (e.g., 'Lat/Lon/Height').
        a: Selector index used for combined time fields. When converting time
            system/format, `a==0` returns the time system code and `a==1`
            returns the time format code.

    Returns:
        Normalized internal code such as 'llh', 'xyz', 'enu', 'nmea', '0'..'2',
        'deg'/'dms', or 'gpst'/'utc'/'jst'/'tow'/'hms'. Empty string if unknown.

    """
    if value == "Lat/Lon/Height":
        return "llh"
    elif value == "X/Y/Z-ECEF":
        return "xyz"
    elif value == "E/N/U-Baseline":
        return "enu"
    elif value == "NMEA 0183":
        return "nmea"
    elif value == "off":
        return "0"
    elif value == "state":
        return "1"
    elif value == "residual":
        return "2"
    elif value == "ddd.ddddddd":
        return "deg"
    elif value == "ddd mm ss.sss":
        return "dms"
    elif value == "ww ssss GPST":
        return "gpst" if a == 0 else "tow"
    elif value == "hh:mm:ss GPST":
        return "gpst" if a == 0 else "hms"
    elif value == "hh:mm:ss UTC":
        return "utc" if a == 0 else "hms"
    elif value == "hh:mm:ss JST":
        return "jst" if a == 0 else "hms"
    else:
        return ""


def _compute_timeformat_display(
    timesys: str | None, timeform: str | None, variables_options=None
) -> str:
    """Return display string for the combined time system/format selection.

    Returns
    -------
    str | None
        Display value such as 'hh:mm:ss GPST', or None if insufficient inputs.

    """
    if not timesys or not timeform:
        return None
    ts = timesys.lower().strip()
    tf = timeform.lower().strip()
    if ts == "gpst":
        return "ww ssss GPST" if tf == "tow" else "hh:mm:ss GPST"
    elif ts == "utc":
        return "hh:mm:ss UTC"
    elif ts == "jst":
        return "hh:mm:ss JST"
    else:
        return "hh:mm:ss GPST"
