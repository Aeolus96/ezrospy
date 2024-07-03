#!/usr/bin/env python3

from ezrospy import ezros_tools
from nicegui import ui

# End of Imports ------------------------------------------------------------------------------------------------------


# Rosboard Setup ------------------------------------------------------------------------------------------------------
ip_address = ezros_tools.get_local_ip()
rosboard_port = 8888
rosboard_url = f"http://{ip_address}:{rosboard_port}/"


# Script Player Setup -------------------------------------------------------------------------------------------------
scripts_directory = ezros_tools.package_path("ezrospy") + "/scripts/"
script_player = ezros_tools.ScriptPlayer(scripts_directory)
script_player.load_files()


# Play/Stop Button ----------------------------------------------------------------------------------------------------
class PlayStopButton(ui.button):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.on("click", self.toggle)

    def toggle(self) -> None:
        """Toggle the button state."""
        self.update()

    def update(self) -> None:
        self.props(
            f'color={"negative" if script_player.process_is_running else "positive"} icon-right={"stop" if script_player.process_is_running else "play_arrow"}'
        )
        super().update()


# GUI Design ----------------------------------------------------------------------------------------------------------
ui.page_title("EzRosPy UI")
ui.add_css("""
    :root {
        --nicegui-default-padding: 0.5rem;
        --nicegui-default-gap: 0.5rem;
    }
""")


with ui.splitter(limits=(30, 80), value=50) as vertical_splitter:
    vertical_splitter.props("").classes("w-full h-screen gap-4")

    with vertical_splitter.separator:
        ui.icon("settings_ethernet", color="primary", size="sm")

    # Left Side - ROSBoard ----------
    with vertical_splitter.before:
        # find ip address
        iframe = ui.element("iframe").style("width:100%; height:100%;")
        iframe._props["src"] = rosboard_url
        with ui.link(target=rosboard_url, new_tab=True).classes("absolute right-0 top-0"):
            ui.button(icon="open_in_new").props("color=grey-2 flat square").classes("w-14 h-14")

    # Right Side - Script Player -----------
    with vertical_splitter.after:
        # Script Selection and Playback Card -----
        with ui.card() as script_card:
            script_card.classes(" w-full grid-cols-5 grid-rows-2")

            async def update_list():
                """Update the dropdown list of script files"""
                ui.notify(script_player.load_files(), type="positive", position="top", timeout=2000)
                file_select_dropdown.options = script_player.file_list

            async def select_file(filename: str) -> None:
                """Select a script file name and load it"""

                ui.notify(f"Script selected: {script_player.file_selected}", position="top", timeout=2000)
                scrolling_log_area.clear()
                with scrolling_log_area:
                    ui.label(f"{script_player.file_selected}:")

            async def handle_file() -> None:
                """Execute the selected file in a separate process"""

                if script_player.process_is_running:  # stop the script
                    ui.notify(script_player.stop_script(), type="negative", position="top", timeout=2000)
                    temp_text = """
                    
                    **********************
                    *** SCRIPT STOPPED ***
                    **********************
                    
                    """
                    print(temp_text)

                else:  # start the script
                    scrolling_log_area.clear()
                    global text_buffer
                    text_buffer = []
                    ui.notify(script_player.execute(), type="positive", position="top", timeout=2000)

            # Dropdown Menu -----
            file_select_dropdown = (
                ui.select(
                    options=script_player.file_list,
                    with_input=True,
                    clearable=True,
                    on_change=lambda e: select_file(e.value),  # Event object with the selected value
                )
                .classes("col-span-full row-span-1")
                .bind_value(script_player, "selected_file")
            )

            # Reload Button -----
            reload_button = (
                ui.button(on_click=update_list).classes(" col-span-1 row-span-1").props(" push no-caps icon=refresh")
            )

            # Start/Stop Button -----
            run_button = (
                PlayStopButton(on_click=handle_file)  # Wrapper around ui.button to enable toggle features
                .classes(" col-span-4 row-span-1")
                .props("push no-caps icon-right=play_arrow")
                .bind_text_from(
                    script_player, "is_running", lambda running: "Stop Script" if running else "Start Script"
                )
            )

        # Script Output Display Card ----------
        with ui.card() as log_card:
            log_card.classes(" w-full h-screen")

            global text_buffer  # Used as Subset of script_player.output_text to keep track of newly appended lines
            text_buffer = []

            def add_label_on_change():
                """Add a new label to scroll area when output_text changes"""
                global text_buffer

                length_difference = len(script_player.process_output_text) - len(text_buffer)
                if length_difference > 0:  # Check if output_text has newly appended lines
                    with scrolling_log_area:
                        for i in range(-length_difference, 0):
                            text = script_player.process_output_text[i]  # Get the newly appended line
                            text_buffer.append(text)
                            ui.label(text).classes("font-mono leading-none whitespace-pre")
                    scrolling_log_area.scroll_to(percent=1.0, duration=0.1)  # Scroll to bottom

                # Update the run button text and color
                run_button.update()

            scrolling_log_area = ui.scroll_area().classes("w-full h-full gap-0 show-scrollbar overflow-y-auto")
            ui.timer(interval=(1 / 60), callback=lambda: add_label_on_change())  # Update log text

# Run GUI -------------------------------------------------------------------------------------------------------------
# NOTE: Needs to be the last element to run the GUI.
ui.run(dark=True, show=False, port=8889)
