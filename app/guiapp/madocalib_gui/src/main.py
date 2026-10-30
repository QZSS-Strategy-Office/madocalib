# Copyright (C) 2026 Cabinet Office, Japan, All rights reserved.
# Copyright (C) 2026 Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
# SPDX-License-Identifier: BSD-2-Clause
"""Application entry point.

This module initializes logging, loads persisted settings, builds the main
Tk window, wires up the UI event bus, and starts the Tk mainloop.
"""

import logging
import sys

from tkinterdnd2 import TkinterDnD

from core.runtime import init_settings, set_window_icon
from ui.eventbus import start as start_ui_bus

# The version displayed on the title bar (kept for backward compatibility).
__version__ = "2.2"
WINDOW_SIZE = "610x395"


def main() -> None:
    """Initialize settings, construct the main window, and start the event loop.

    Notes:
        Uses TkinterDnD for drag-and-drop and runs event bus polling at 33 ms.

    """
    # Logging config
    root_logger = logging.getLogger()
    if not root_logger.handlers:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
            stream=sys.stdout,
        )

    # Load config
    init_settings()

    # Create main window
    main_window = TkinterDnD.Tk()
    main_window.withdraw()
    main_window.title(f"madocalib v{__version__}")
    main_window.geometry(WINDOW_SIZE)

    # Defer import to avoid circular references.
    from ui.view_main import views

    views(main_window)

    set_window_icon(main_window)
    start_ui_bus(main_window, interval_ms=33)
    main_window.deiconify()
    main_window.mainloop()


if __name__ == "__main__":
    main()
