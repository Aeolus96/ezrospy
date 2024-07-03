from nicegui import ui
from nicegui.events import KeyEventArguments


# Change some defaults for NiceGUI ------------------------------------------------------------------------------------
ui.add_css("""
    :root {
        --nicegui-default-padding: 0.5rem;
        --nicegui-default-gap: 0.5rem;
    }
""")

# https://www.gradient-animator.com/

# Purple Pink:
# background: linear-gradient(270deg, #d630ea, #ea30a1, #7930ea);
# Blue:
# background: linear-gradient(45deg, #1a2980, #26d0ce);
# Red Orange:
# background: linear-gradient(270deg, #ea8430, #d92020);
# Purple Peach:
# background: linear-gradient(43deg, #FFCC70, #C850C0, #4158D0);
# Dark Theme:
# background: linear-gradient(270deg, #4b79a1, #283e51);

background = """
    background: linear-gradient(270deg, #ea8430, #d92020);
    background-size: 600% 600%;

    -webkit-animation: AnimationName 59s ease infinite;
    -moz-animation: AnimationName 59s ease infinite;
    animation: AnimationName 59s ease infinite;
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


# GUI Page Setup ------------------------------------------------------------------------------------------------------
@ui.page("/", title="EzRosPy UI")
def index():
    # Set background color
    # ui.query("body").classes("bg-gradient-to-r from-rose-700 to-orange-500")
    ui.query("body").style(f"{background}")
    ui.add_css(f"{gradient}")

    def handle_key(e: KeyEventArguments):
        if e.modifiers.shift and e.key.enter and e.action.keydown:
            ui.notify("playing")

    keyboard = ui.keyboard(on_key=handle_key, active=True)

    # Overall Card ----------------------------------------------------------------------------------------------------
    with ui.card().tight() as page_card:
        page_card.classes("rounded-xl shadow-lg shadow-black bg-white/30")
        page_card.style(
            "width: 95vw; height: 95vh; position: absolute; top: 50%; left: 50%; -ms-transform: translate(-50%, -50%); transform: translate(-50%, -50%);"
        )

        # Script Player Section ---------------------------------------------------------------------------------------
        with ui.card_section().classes("w-full p-0 shadow-lg shadow-black bg-white/30 flex justify-evenly"):
            ui.button().props("flat round icon=refresh size=md color=dark").classes("m-2")
            file_select_dropdown = (
                ui.select(
                    label="File",
                    options=["Select Script"],
                    with_input=True,
                    clearable=True,
                )
                .classes("grow my-2")
                .props("standout dense options-dense options-dark rounded")  # add loading maybe
            )
            ui.button().props("flat round icon=play_arrow size=md color=dark").classes("m-2")

        # Split Section with 2 columns --------------------------------------------------------------------------------
        with ui.splitter(limits=(30, 70), value=50) as vertical_splitter:
            vertical_splitter.classes("w-full h-full gap-0")

            with vertical_splitter.separator:
                ui.card().classes("w-1 h-4/6 p-0 m-0 opacity-70")

            # Left Side - Rosboard --------------------------------------------
            with vertical_splitter.before:
                # iframe = ui.element("iframe").style("width:100%; height:100%;")
                # iframe._props["src"] = "http://127.0.0.1:8889/"
                with ui.link(
                    target="https://github.com/Aeolus96/actor_ros/blob/main/scripts/actor_gui.py", new_tab=True
                ).classes("absolute right-0 top-0"):
                    ui.button(icon="open_in_new").props("color=dark flat square").classes("w-14 h-14")

            # Right Side - Script Output --------------------------------------
            with vertical_splitter.after:
                with ui.card_section().classes("w-full h-full bg-black/10 rounded-br-xl"):
                    scrolling_log_area = ui.scroll_area().classes("w-full h-full gap-0 show-scrollbar overflow-y-auto")


ui.run(show=False, port=8889)
