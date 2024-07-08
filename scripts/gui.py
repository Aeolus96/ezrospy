#!/usr/bin/env python3

from ezrospy import ezros_tools
from nicegui import ui
from nicegui.events import KeyEventArguments

# End of Imports ------------------------------------------------------------------------------------------------------


# Rosboard Setup ------------------------------------------------------------------------------------------------------
ip_address = ezros_tools.get_local_ip()
rosboard_port = 8888
rosboard_url = f"http://{ip_address}:{rosboard_port}/"


# Script Player Setup -------------------------------------------------------------------------------------------------
scripts_directory = ezros_tools.package_path("ezrospy") + "/scripts/"
script_player = ezros_tools.ScriptPlayer(scripts_directory)
script_player.load_files()
global text_buffer  # Tracks newly appended lines in script_player.output_text
text_buffer = []


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


# Key Events ----------------------------------------------------------------------------------------------------------
def handle_playback_key(e: KeyEventArguments):  # Handle keyboard events for playback
    if e.modifiers.shift and e.key.enter and e.action.keydown:
        ui.notify("playing")  # Link to button later


# GUI Setup -----------------------------------------------------------------------------------------------------------
ui.add_css("""
    :root {
        --nicegui-default-padding: 0.5rem;
        --nicegui-default-gap: 0.5rem;
    }
""")
background = """
    background: linear-gradient(270deg, #ffe61f, #ff782a, #ffb731);
    background-size: 600% 600%;

    -webkit-animation: AnimationName 200s ease infinite;
    -moz-animation: AnimationName 200s ease infinite;
    animation: AnimationName 200s ease infinite;
"""
gradient = """
    @-webkit-keyframes AnimationName {
        0%{background-position:0% 50%}
        50%{background-position:100% 50%}
        100%{background-position:0% 50%}
    }
    @-moz-keyframes AnimationName {
        0%{background-position:0% 50%}
        50%{background-position:100% 50%}
        100%{background-position:0% 50%}
    }
    @-o-keyframes AnimationName {
        0%{background-position:0% 50%}
        50%{background-position:100% 50%}
        100%{background-position:0% 50%}
    }
    @keyframes AnimationName {
        0%{background-position:0% 50%}
        50%{background-position:100% 50%}
        100%{background-position:0% 50%}
    }
"""


@ui.page("/", title="EzRosPy UI")  # Set the page title and path
def index():
    ui.query("body").style(f"{background}")  # Set the background
    ui.add_css(f"{gradient}")  # Animate the background

    # Main Card -------------------------------------------------------------------------------------------------------
    with ui.card().tight() as page_card:
        page_card.classes("rounded-xl shadow-lg shadow-black bg-white/30")
        page_card.style(
            "width: 95vw; height: 95vh; position: absolute; top: 50%; left: 50%; -ms-transform: translate(-50%, -50%); transform: translate(-50%, -50%);"
        )

        # Script Player Section ---------------------------------------------------------------------------------------
        with ui.card_section().classes("w-full p-0 shadow-lg shadow-black bg-white/30 flex justify-evenly"):
            # Script related methods ------------------------------------------
            async def update_list() -> None:
                """Update the dropdown list of script files"""
                ui.notify(script_player.load_files(), type="positive", timeout=2000)
                file_select_dropdown.options = script_player.file_list

            async def select_file(filename: str) -> None:
                """Select a script file name and load it"""
                ui.notify(f"GUI: Script selected__{script_player.file_selected}__", timeout=2000)
                scrolling_log_area.clear()
                with scrolling_log_area:
                    ui.label(f"{script_player.file_selected}:")

            async def handle_file() -> None:
                """Execute the selected file in a separate process"""
                if script_player.process_is_running:  # Stop the script
                    ui.notify(script_player.stop_script(), type="negative", timeout=2000)
                    temp_text = """
                    
                    **********************
                    *** SCRIPT STOPPED ***
                    **********************
                    
                    """
                    print(temp_text)
                else:  # Start the script
                    scrolling_log_area.clear()
                    global text_buffer
                    text_buffer = []
                    ui.notify(script_player.execute(), type="positive", timeout=2000)

            def add_label_on_change():
                """Add a new label to scroll area when output_text changes"""
                global text_buffer
                length_difference = len(script_player.process_output_text) - len(text_buffer)
                if length_difference > 0:  # Check if output_text has newly appended lines
                    with scrolling_log_area:
                        for i in range(-length_difference, 0):
                            text = script_player.process_output_text[i]  # Get the newly appended line
                            text_buffer.append(text)
                            ui.label(text).classes("font-mono text-xs leading-none whitespace-pre antialiased")
                    scrolling_log_area.scroll_to(percent=1.0, duration=0.1)  # Scroll to bottom
                run_button.update()

            # Script related methods ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

            ui.button(on_click=update_list).props("flat round icon=refresh size=md color=dark").classes("m-2")

            file_select_dropdown = (
                ui.select(
                    label="File",
                    options=script_player.file_list,
                    with_input=True,
                    clearable=True,
                    on_change=lambda e: select_file(e.value),
                )
                .classes("grow my-2")
                .props("standout dense options-dense options-dark rounded")  # add loading maybe
                .bind_value(script_player, "file_selected")
            )
            run_button = (
                PlayStopButton(on_click=handle_file)
                .props("flat round icon=play_arrow size=md color=dark")
                .classes("m-2")
                # .bind_text_from(
                #     script_player, "process_is_running", lambda running: "Stop Script" if running else "Start Script"
                # )
            )
            ui.keyboard(on_key=handle_playback_key, active=True)  # Keyboard controls for the run_button

        # Split Section with 2 columns --------------------------------------------------------------------------------
        with ui.splitter(limits=(30, 70), value=30) as vertical_splitter:
            vertical_splitter.classes("w-full h-full gap-0")

            with vertical_splitter.separator:
                ui.card().classes("w-1 h-4/6 p-0 m-0 opacity-70")

            # Left Side - Rosboard --------------------------------------------
            with vertical_splitter.before:
                iframe = ui.element("iframe").style("width:100%; height:100%;")
                iframe._props["src"] = rosboard_url
                with ui.link(target=rosboard_url, new_tab=True).classes("absolute right-0 top-0"):
                    ui.button(icon="open_in_new").props("color=dark flat square").classes("w-14 h-14")

            # Right Side - Script Output --------------------------------------
            with vertical_splitter.after:
                with ui.card_section().classes("w-full h-full bg-black/10 rounded-br-xl"):
                    scrolling_log_area = ui.scroll_area().classes("w-full h-full gap-0 show-scrollbar overflow-y-auto")
                    ui.timer(interval=(1 / 60), callback=lambda: add_label_on_change())  # Update log text


# Run GUI -------------------------------------------------------------------------------------------------------------

ui.run(show=False, port=8889)

# ---------------------------------------------------------------------------------------------------------------------
