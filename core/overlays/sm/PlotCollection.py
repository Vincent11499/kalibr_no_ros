"""Kalibr plot tabs with an optional wxPython runtime dependency."""

from __future__ import print_function

import collections


try:
    import wx
    import wx.aui
    from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as Canvas
    from matplotlib.backends.backend_wx import NavigationToolbar2Wx as Toolbar
except ImportError:
    wx = None
    Canvas = None
    Toolbar = None


class PlotCollection:
    def __init__(self, window_name="", window_size=(800, 600)):
        self.frame_name = window_name
        self.window_size = window_size
        self.figureList = collections.OrderedDict()

    def add_figure(self, tabname, fig):
        self.figureList[tabname] = fig

    def delete_figure(self, name):
        self.figureList.pop(name, None)

    def show(self):
        if not self.figureList:
            return
        if wx is None:
            raise RuntimeError(
                "Displaying Kalibr report windows requires wxPython. "
                "Install python3-wxgtk4.0 or run with --dont-show-report."
            )
        app = wx.App()
        frame = wx.Frame(None, -1, self.frame_name, size=self.window_size)
        plotter = self.PlotNotebook(frame)
        for name, figure in self.figureList.items():
            plotter.add(name, figure)
        frame.Show()
        app.MainLoop()

    if wx is not None:
        class Plot(wx.Panel):
            def __init__(self, parent, fig, id=-1, dpi=None, **kwargs):
                del dpi
                wx.Panel.__init__(self, parent, id=id, **kwargs)
                fig.set_figheight(2)
                fig.set_figwidth(2)
                self.canvas = Canvas(self, -1, fig)
                self.toolbar = Toolbar(self.canvas)
                self.toolbar.Realize()

                sizer = wx.BoxSizer(wx.VERTICAL)
                sizer.Add(self.canvas, 1, wx.EXPAND)
                sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
                self.SetSizer(sizer)

        class PlotNotebook(wx.Panel):
            def __init__(self, parent, id=-1):
                wx.Panel.__init__(self, parent, id=id)
                self.nb = wx.aui.AuiNotebook(self)
                sizer = wx.BoxSizer()
                sizer.Add(self.nb, 1, wx.EXPAND)
                self.SetSizer(sizer)

            def add(self, name, fig):
                page = PlotCollection.Plot(self.nb, fig)
                self.nb.AddPage(page, name)
