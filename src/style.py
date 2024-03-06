from gradio.themes.utils import colors, fonts, sizes
from gradio.themes.base import Base
from typing import Iterable

darkRed = colors.Color(
    name="darkRed",
    c50="#ffb3b3",
    c100="#ff8080",
    c200="#ff4d4d",
    c300="#ff1a1a",
    c400="#e60000",
    c500="#b30000",
    c600="#800000",
    c700="#4d0000",
    c800="#1a0000",
    c900="#000000",
    c950="#000000", 
)


class CapraStyle(Base):
    def __init__(
        self,
        *,
        primary_hue: colors.Color | str = colors.red,  # Changed to red
        secondary_hue: colors.Color | str = darkRed,  # Dark red for contrast
        neutral_hue: colors.Color | str = colors.slate,  # Slate for a decent look
        spacing_size: sizes.Size | str = sizes.spacing_md,
        radius_size: sizes.Size | str = sizes.radius_md,
        text_size: sizes.Size | str = sizes.text_lg,
        font: fonts.Font
        | str
        | Iterable[fonts.Font | str] = (
            fonts.GoogleFont("Roboto"),  # Roboto for clear readability
            "ui-sans-serif",
            "sans-serif",
        ),
        font_mono: fonts.Font
        | str
        | Iterable[fonts.Font | str] = (
            fonts.GoogleFont("IBM Plex Mono"),
            "ui-monospace",
            "monospace",
        ),
    ):
        super().__init__(
            primary_hue=primary_hue,
            secondary_hue=secondary_hue,
            neutral_hue=neutral_hue,
            spacing_size=spacing_size,
            radius_size=radius_size,
            text_size=text_size,
            font=font,
            font_mono=font_mono,
        )
