# -*- coding: utf-8 -*-
"""A small layout sketch editor for the Step42 desktop controller.

This is intentionally a wireframe tool.  It saves the product layout and
control intent to JSON; the production serial application remains hand-built.
"""

import json
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


CANVAS_WIDTH = 1180
CANVAS_HEIGHT = 720
DEFAULT_LAYOUT = "upper_computer_layout.json"
PAGES = ("标准界面", "高级调参", "工厂测试")

WIDGET_DEFAULTS = {
    "标题": ("标题", 40, 30, 360, 34, 18),
    "分组框": ("新分组", 40, 80, 340, 180, 12),
    "文本": ("状态：--", 60, 110, 220, 24, 11),
    "按钮": ("执行", 60, 150, 96, 30, 11),
    "输入框": ("数值", 60, 195, 130, 28, 11),
    "下拉框": ("选项", 60, 235, 150, 28, 11),
    "复选框": ("启用选项", 60, 275, 150, 24, 11),
    "分隔线": ("", 60, 315, 260, 2, 10),
    "波形区": ("角度 / 目标 / 误差", 430, 80, 520, 260, 12),
    "页签": ("标准 | 高级 | 工厂", 40, 20, 430, 32, 11),
}


class LayoutEditor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Step42 上位机布局草图编辑器")
        self.geometry("1540x880")
        self.minsize(1240, 760)

        self.items = []
        self.selected_id = None
        self.canvas_items = {}
        self.drag_offset = (0, 0)
        self.next_id = 1

        self.page_var = tk.StringVar(value=PAGES[0])
        self.type_var = tk.StringVar(value="分组框")
        self.status_var = tk.StringVar(value="已准备：添加控件后拖动它们。")
        self._make_property_vars()
        self._build_ui()
        self._add_starter_layout()

    def _make_property_vars(self):
        self.prop_vars = {
            "type": tk.StringVar(),
            "page": tk.StringVar(),
            "text": tk.StringVar(),
            "action": tk.StringVar(),
            "options": tk.StringVar(),
            "x": tk.StringVar(),
            "y": tk.StringVar(),
            "w": tk.StringVar(),
            "h": tk.StringVar(),
            "font_size": tk.StringVar(),
        }

    def _build_ui(self):
        toolbar = ttk.Frame(self, padding=(10, 8))
        toolbar.pack(fill=tk.X)
        ttk.Label(toolbar, text="当前页面").pack(side=tk.LEFT)
        page = ttk.Combobox(toolbar, textvariable=self.page_var, values=PAGES,
                            state="readonly", width=12)
        page.pack(side=tk.LEFT, padx=(6, 16))
        page.bind("<<ComboboxSelected>>", lambda _event: self.redraw())

        ttk.Label(toolbar, text="新控件").pack(side=tk.LEFT)
        ttk.Combobox(toolbar, textvariable=self.type_var,
                     values=tuple(WIDGET_DEFAULTS.keys()), state="readonly", width=10).pack(
                         side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="添加", command=self.add_item).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="复制", command=self.duplicate_selected).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(toolbar, text="删除", command=self.delete_selected).pack(side=tk.LEFT, padx=(6, 18))
        ttk.Button(toolbar, text="保存 JSON", command=self.save_layout).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="打开 JSON", command=self.load_layout).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="清空当前页", command=self.clear_current_page).pack(side=tk.LEFT)

        content = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        content.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 8))

        left = ttk.Frame(content, padding=(0, 0, 10, 0))
        content.add(left, weight=1)
        self.canvas = tk.Canvas(left, width=CANVAS_WIDTH, height=CANVAS_HEIGHT,
                                background="#f3f5f7", highlightthickness=1,
                                highlightbackground="#aab2bd")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.bind("<Delete>", lambda _event: self.delete_selected())

        right = ttk.LabelFrame(content, text="控件属性", padding=10)
        content.add(right, weight=0)
        self._build_properties(right)

        ttk.Label(self, textvariable=self.status_var, anchor=tk.W,
                  padding=(12, 5)).pack(fill=tk.X)

    def _build_properties(self, parent):
        row = 0
        for label, key in (("类型", "type"), ("页面", "page")):
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=(0, 5))
            ttk.Label(parent, textvariable=self.prop_vars[key], width=22).grid(
                row=row, column=1, sticky=tk.W, pady=(0, 5))
            row += 1

        for label, key in (("显示文字", "text"), ("功能标识", "action"),
                           ("下拉选项", "options")):
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=(4, 2))
            entry = ttk.Entry(parent, textvariable=self.prop_vars[key], width=25)
            entry.grid(row=row, column=1, sticky=tk.EW, pady=(4, 2))
            entry.bind("<Return>", lambda _event: self.apply_properties())
            row += 1

        ttk.Separator(parent).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
        row += 1
        for label, key in (("X", "x"), ("Y", "y"), ("宽", "w"), ("高", "h"), ("字号", "font_size")):
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=3)
            ttk.Entry(parent, textvariable=self.prop_vars[key], width=10).grid(
                row=row, column=1, sticky=tk.W, pady=3)
            row += 1

        ttk.Button(parent, text="应用属性", command=self.apply_properties).grid(
            row=row, column=0, columnspan=2, sticky=tk.EW, pady=(14, 6))
        row += 1
        ttk.Label(parent, text=("功能标识示例：\n"
                                "connect_debug\nstream_toggle\ntrack_pid_write\n"
                                "position_send\nstatus_query"), justify=tk.LEFT,
                  foreground="#59636e").grid(row=row, column=0, columnspan=2, sticky=tk.W)
        parent.columnconfigure(1, weight=1)

    def _add_starter_layout(self):
        starters = [
            ("页签", "Step42 控制中心  |  标准  高级  工厂", "", 28, 18, 620, 34),
            ("分组框", "连接与通信", "", 28, 70, 340, 142),
            ("下拉框", "COM端口", "port_select", 48, 108, 140, 28),
            ("下拉框", "Error / Debug", "interface_select", 200, 108, 140, 28),
            ("按钮", "连接", "connect", 48, 152, 90, 30),
            ("按钮", "读取状态", "status_query", 150, 152, 110, 30),
            ("分组框", "内置误差 PID", "", 28, 235, 340, 250),
            ("输入框", "Kp", "track_pid_kp", 48, 280, 130, 28),
            ("输入框", "Ki", "track_pid_ki", 48, 322, 130, 28),
            ("输入框", "Kd", "track_pid_kd", 48, 364, 130, 28),
            ("按钮", "写入 PID", "track_pid_write", 48, 418, 110, 30),
            ("按钮", "恢复默认", "track_pid_default", 170, 418, 110, 30),
            ("波形区", "目标 / 当前角度 / 视觉误差", "telemetry_plot", 402, 70, 680, 415),
            ("分组框", "定位与速度", "", 402, 510, 680, 155),
            ("输入框", "目标角度", "position_angle", 424, 554, 130, 28),
            ("按钮", "发送角度", "position_send", 566, 554, 110, 30),
            ("复选框", "开启数据流", "stream_toggle", 710, 556, 130, 24),
        ]
        for widget_type, text, action, x, y, width, height in starters:
            self._new_item(widget_type, text, action, x, y, width, height)
        self.redraw()

    def _new_item(self, widget_type, text=None, action="", x=None, y=None, width=None, height=None):
        default_text, dx, dy, dw, dh, font_size = WIDGET_DEFAULTS[widget_type]
        item = {
            "id": f"item_{self.next_id}",
            "type": widget_type,
            "page": self.page_var.get(),
            "text": default_text if text is None else text,
            "action": action,
            "options": "",
            "x": dx if x is None else x,
            "y": dy if y is None else y,
            "w": dw if width is None else width,
            "h": dh if height is None else height,
            "font_size": font_size,
        }
        self.next_id += 1
        self.items.append(item)
        return item

    def add_item(self):
        item = self._new_item(self.type_var.get())
        item["x"] += (len(self.items) % 5) * 18
        item["y"] += (len(self.items) % 4) * 14
        self.select(item["id"])
        self.redraw()

    def current_item(self):
        return next((item for item in self.items if item["id"] == self.selected_id), None)

    def select(self, item_id):
        self.selected_id = item_id
        item = self.current_item()
        if item is None:
            for var in self.prop_vars.values():
                var.set("")
            return
        for key, var in self.prop_vars.items():
            var.set(str(item.get(key, "")))
        self.status_var.set(f"已选中 {item['type']}：{item['text'] or item['id']}")

    def delete_selected(self):
        if self.selected_id is None:
            return
        self.items = [item for item in self.items if item["id"] != self.selected_id]
        self.select(None)
        self.redraw()

    def duplicate_selected(self):
        item = self.current_item()
        if item is None:
            return
        clone = dict(item)
        clone["id"] = f"item_{self.next_id}"
        self.next_id += 1
        clone["x"] += 20
        clone["y"] += 20
        self.items.append(clone)
        self.select(clone["id"])
        self.redraw()

    def clear_current_page(self):
        page = self.page_var.get()
        if not messagebox.askyesno("清空页面", f"删除“{page}”的全部控件？"):
            return
        self.items = [item for item in self.items if item["page"] != page]
        self.select(None)
        self.redraw()

    def apply_properties(self):
        item = self.current_item()
        if item is None:
            return
        try:
            for key in ("x", "y", "w", "h", "font_size"):
                item[key] = max(1, int(self.prop_vars[key].get()))
        except ValueError:
            messagebox.showerror("数值无效", "X、Y、宽、高和字号必须是整数。")
            return
        for key in ("text", "action", "options"):
            item[key] = self.prop_vars[key].get()
        self.redraw()

    def _draw_item(self, item):
        x, y, w, h = item["x"], item["y"], item["w"], item["h"]
        item_type = item["type"]
        text = item["text"]
        font = ("Microsoft YaHei UI", item["font_size"])
        tags = ("layout_item", item["id"])

        if item_type == "分组框":
            self.canvas.create_rectangle(x, y, x + w, y + h, outline="#687789", width=1, tags=tags)
            self.canvas.create_rectangle(x + 10, y - 8, x + 18 + len(text) * 12, y + 8,
                                         fill="#f3f5f7", outline="", tags=tags)
            self.canvas.create_text(x + 14, y, text=text, anchor=tk.W, font=font,
                                    fill="#263241", tags=tags)
        elif item_type == "按钮":
            self.canvas.create_rectangle(x, y, x + w, y + h, fill="#2474b5", outline="#155681", tags=tags)
            self.canvas.create_text(x + w / 2, y + h / 2, text=text, font=font,
                                    fill="white", tags=tags)
        elif item_type in ("输入框", "下拉框"):
            self.canvas.create_rectangle(x, y, x + w, y + h, fill="white", outline="#9da8b5", tags=tags)
            label = text + ("  v" if item_type == "下拉框" else "")
            self.canvas.create_text(x + 8, y + h / 2, text=label, anchor=tk.W, font=font,
                                    fill="#263241", tags=tags)
        elif item_type == "复选框":
            self.canvas.create_rectangle(x, y + 4, x + 16, y + 20, fill="white", outline="#687789", tags=tags)
            self.canvas.create_text(x + 24, y + 12, text=text, anchor=tk.W, font=font,
                                    fill="#263241", tags=tags)
        elif item_type == "分隔线":
            self.canvas.create_line(x, y, x + w, y, fill="#aab2bd", width=max(1, h), tags=tags)
        elif item_type == "波形区":
            self.canvas.create_rectangle(x, y, x + w, y + h, fill="#16212b", outline="#455768", tags=tags)
            self.canvas.create_text(x + 12, y + 12, text=text, anchor=tk.W, font=font,
                                    fill="#e2edf7", tags=tags)
            for index in range(1, 5):
                gy = y + h * index / 5
                self.canvas.create_line(x + 8, gy, x + w - 8, gy, fill="#304454", tags=tags)
            self.canvas.create_line(x + 12, y + h * 0.66, x + w * 0.38, y + h * 0.45,
                                    x + w * 0.65, y + h * 0.7, x + w - 12, y + h * 0.32,
                                    fill="#42c98c", width=2, smooth=True, tags=tags)
        elif item_type == "页签":
            self.canvas.create_rectangle(x, y, x + w, y + h, fill="#e4eaf0", outline="#b3bfcb", tags=tags)
            self.canvas.create_text(x + 12, y + h / 2, text=text, anchor=tk.W, font=font,
                                    fill="#263241", tags=tags)
        else:
            self.canvas.create_text(x, y, text=text, anchor=tk.NW, font=font,
                                    fill="#263241", tags=tags)

        if item["id"] == self.selected_id:
            self.canvas.create_rectangle(x - 3, y - 3, x + w + 3, y + h + 3,
                                         outline="#ed8a24", width=2, dash=(4, 2),
                                         tags=("selection", item["id"]))

    def redraw(self):
        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT, fill="#f3f5f7", outline="")
        self.canvas.create_text(12, CANVAS_HEIGHT - 14,
                                text=f"布局尺寸 {CANVAS_WIDTH} x {CANVAS_HEIGHT} | 当前页面：{self.page_var.get()}",
                                anchor=tk.W, fill="#687789", font=("Microsoft YaHei UI", 9))
        self.canvas_items.clear()
        for item in self.items:
            if item["page"] == self.page_var.get():
                self._draw_item(item)

    def _item_id_from_event(self, event):
        found = self.canvas.find_withtag("current")
        if not found:
            return None
        tags = self.canvas.gettags(found[0])
        return next((tag for tag in tags if tag.startswith("item_")), None)

    def on_canvas_click(self, event):
        item_id = self._item_id_from_event(event)
        self.select(item_id)
        item = self.current_item()
        if item is not None:
            self.drag_offset = (event.x - item["x"], event.y - item["y"])
        self.redraw()

    def on_canvas_drag(self, event):
        item = self.current_item()
        if item is None:
            return
        item["x"] = max(0, event.x - self.drag_offset[0])
        item["y"] = max(0, event.y - self.drag_offset[1])
        self.prop_vars["x"].set(str(item["x"]))
        self.prop_vars["y"].set(str(item["y"]))
        self.redraw()

    def on_canvas_release(self, _event):
        item = self.current_item()
        if item is not None:
            self.status_var.set(f"已移动“{item['text'] or item['id']}”，可继续在右侧调整精确尺寸。")

    def export_data(self):
        return {
            "format": "step42-ui-layout-v1",
            "canvas": {"width": CANVAS_WIDTH, "height": CANVAS_HEIGHT},
            "items": self.items,
        }

    def save_layout(self):
        path = filedialog.asksaveasfilename(
            title="保存布局", defaultextension=".json", initialfile=DEFAULT_LAYOUT,
            filetypes=(("JSON 布局", "*.json"), ("所有文件", "*.*")))
        if not path:
            return
        with open(path, "w", encoding="utf-8") as file:
            json.dump(self.export_data(), file, ensure_ascii=False, indent=2)
        self.status_var.set(f"已保存布局：{os.path.basename(path)}")

    def load_layout(self):
        path = filedialog.askopenfilename(
            title="打开布局", filetypes=(("JSON 布局", "*.json"), ("所有文件", "*.*")))
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as file:
                data = json.load(file)
            if data.get("format") != "step42-ui-layout-v1":
                raise ValueError("不是 Step42 布局文件")
            self.items = data.get("items", [])
            highest = max((int(item["id"].split("_")[-1]) for item in self.items), default=0)
            self.next_id = highest + 1
            self.select(None)
            self.redraw()
            self.status_var.set(f"已打开布局：{os.path.basename(path)}")
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
            messagebox.showerror("打开失败", str(error))


if __name__ == "__main__":
    LayoutEditor().mainloop()
