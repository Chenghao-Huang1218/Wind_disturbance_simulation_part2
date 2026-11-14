"""Redraw flat status areas while leaving widgets and exported figures intact."""

from __future__ import annotations

from typing import Sequence

from matplotlib.backend_bases import DrawEvent, ResizeEvent
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from matplotlib.transforms import Bbox
from matplotlib.text import Text

Region = tuple[tuple[float, float, float, float], str, Sequence[Text]]


class StatusRenderer:
    """Paint only widget-free, solid-background regions of a regular figure.

    Artists remain ordinary, nonanimated artists: savefig and widget-specific
    blits work normally. Region coordinates are relative to the figure.

    Args:
        figure: Figure whose canvas supports optional partial blitting.
        regions: Background rectangle, fill color and status artists per region.
    """

    def __init__(self, figure: Figure, regions: Sequence[Region]) -> None:
        self.figure = figure
        self.regions = []
        self.ready = False
        self._regions_safe = False
        self._last_states: dict[int, tuple[tuple[str, bool], ...]] = {}
        for bounds, color, artists in regions:
            patch = Rectangle(bounds[:2], bounds[2], bounds[3],
                              facecolor=color, edgecolor="none",
                              transform=figure.transFigure)
            patch.set_figure(figure)
            self.regions.append((bounds, patch, artists))
        figure.canvas.mpl_connect("draw_event", self._on_draw)
        figure.canvas.mpl_connect("resize_event", self._on_resize)

    def _on_draw(self, event: DrawEvent) -> None:
        """A save may switch renderer/DPI; wait for a normal draw afterwards."""
        self.ready = (event.canvas is self.figure.canvas
                      and not event.canvas.is_saving())
        managed = {artist for _, _, artists in self.regions for artist in artists}
        protected = [artist.get_window_extent(event.renderer)
                     for artist in self.figure.texts
                     if artist not in managed and artist.get_visible()]
        protected.extend(axes.get_window_extent(event.renderer)
                         for axes in self.figure.axes if axes.get_visible())
        self._regions_safe = all(
            not any(self.figure.transFigure.transform_bbox(
                Bbox.from_bounds(*bounds)).overlaps(box) for box in protected)
            for bounds, _, _ in self.regions
        )
        self._regions_safe = self._regions_safe and all(
            self._fits(self.figure.transFigure.transform_bbox(
                Bbox.from_bounds(*bounds)), artist.get_window_extent(event.renderer))
            for bounds, _, artists in self.regions for artist in artists
            if artist.get_visible()
        )
        self._last_states = {
            index: tuple((artist.get_text(), artist.get_visible()) for artist in artists)
            for index, (_, _, artists) in enumerate(self.regions)
        } if self.ready else {}

    def _on_resize(self, event: ResizeEvent) -> None:
        """Require a normal full draw after the canvas dimensions change."""
        self.ready = False
        self._regions_safe = False
        self._last_states.clear()

    def draw(self) -> None:
        """Refresh status regions, or fall back safely on unsupported canvases."""
        canvas = self.figure.canvas
        # Point-sized text can cross fractional region bounds after resizing.
        # Check actual artist bounds, allowing harmless Tk pixel rounding.
        if not canvas.supports_blit or not self.ready or not self._regions_safe:
            canvas.draw_idle()
            return
        updates = []
        renderer = canvas.get_renderer()
        for index, (bounds, patch, artists) in enumerate(self.regions):
            state = tuple((artist.get_text(), artist.get_visible()) for artist in artists)
            if state == self._last_states.get(index):
                continue
            bbox = self.figure.transFigure.transform_bbox(Bbox.from_bounds(*bounds))
            if any(not self._fits(bbox, artist.get_window_extent(renderer))
                   for artist in artists if artist.get_visible()):
                canvas.draw_idle()
                return
            updates.append((index, bbox, patch, artists, state))
        for index, bbox, patch, artists, state in updates:
            self.figure.draw_artist(patch)
            for artist in artists:
                if artist.get_visible():
                    self.figure.draw_artist(artist)
            canvas.blit(bbox)
            self._last_states[index] = state

    @staticmethod
    def _fits(region: Bbox, text: Bbox) -> bool:
        """Include glyph edge padding when deciding whether erasure is safe."""
        padded = text.padded(1.)
        return (region.x0 <= padded.x0 and region.y0 <= padded.y0
                and region.x1 >= padded.x1 and region.y1 >= padded.y1)
