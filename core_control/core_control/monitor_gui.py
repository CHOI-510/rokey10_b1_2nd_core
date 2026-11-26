"""Independent Tk dashboard for the Core Project ROS graph."""

from datetime import datetime
import signal
from threading import Thread
from time import monotonic
import tkinter as tk
from tkinter import font, ttk

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from core_common.ros_context import ros_context
from core_common.status_qos import current_stamp
from core_common.types import State
from .monitor_data import COMMANDS, MonitorNode

BG, CARD, NAVY = '#e9f0f7', '#ffffff', '#17324d'
MUTED, TRACK = '#48647f', '#e9f0f7'
BLUE, TEAL, RED = '#245fab', '#087d83', '#b42d44'
GREEN, AMBER = '#087052', '#855513'
STAGES = tuple(state.value for state in State if state not in (State.STOPPED, State.RECOVER))
OPTIONAL = {'FETCH', 'WAIT_WORKER', 'RECHECK'}
AFTER_BRANCH = {'STOW', 'FASTEN', 'VERIFY'}
COLORS = {'PASS': GREEN, 'FAIL': RED, 'HOLD': AMBER, 'STOPPED': RED,
          'PRESENT': GREEN, 'MISSING': AMBER, 'PROTRUDING': AMBER,
          'NOT_PROTRUDING': GREEN, 'SEATED': GREEN, 'NOT_SEATED': RED, 'UNKNOWN': AMBER}


def rounded(canvas, x, y, width, height, radius, **style):
    right, bottom = x + width, y + height
    points = (x + radius, y, right - radius, y, right, y, right, y + radius,
              right, bottom - radius, right, bottom, right - radius, bottom,
              x + radius, bottom, x, bottom, x, bottom - radius, x, y + radius, x, y)
    return canvas.create_polygon(points, smooth=True, splinesteps=16, **style)


def label(parent, text='', *, bg=CARD, fg=NAVY, size=14, bold=False, **options):
    return tk.Label(parent, text=text, bg=bg, fg=fg,
                    font=('TkDefaultFont', -size, 'bold' if bold else 'normal'), **options)


class Card(tk.Canvas):
    def __init__(self, parent, title):
        super().__init__(parent, bg=BG, highlightthickness=0, width=1, height=1)
        self.body = tk.Frame(self, bg=CARD)
        self.content = self.create_window(13, 10, window=self.body, anchor='nw')
        label(self.body, title, size=17, bold=True).pack(anchor='w', pady=(0, 8))
        self.bind('<Configure>', self.layout)

    def layout(self, event):
        self.delete('surface')
        shape = rounded(self, 1, 1, event.width - 2, event.height - 2, 15,
                        fill=CARD, outline='#d4e0ec', tags='surface')
        self.tag_lower(shape)
        self.itemconfigure(self.content, width=max(1, event.width - 26),
                           height=max(1, event.height - 20))


class CameraPanel(tk.Canvas):
    def __init__(self, parent, title):
        super().__init__(parent, bg=CARD, highlightthickness=0, width=1, height=1)
        self.title = title
        self.displayed = None
        self.photo = None

    def render(self, status, frame):
        width, height = self.winfo_width(), self.winfo_height()
        key = (status, frame[1] if frame else None, width, height)
        if key == self.displayed:
            return
        self.displayed = key
        self.delete('all')
        if width < 30 or height < 56:
            return
        rounded(self, 0, 0, width, height, 11, fill=NAVY)
        if frame:
            picture = frame[0].copy()
            picture.thumbnail((width - 10, height - 40))
            picture = picture.convert('RGB')
            ppm = f'P6 {picture.width} {picture.height} 255\n'.encode() + picture.tobytes()
            self.photo = tk.PhotoImage(data=ppm, format='PPM', master=self)
            self.create_image(width / 2, (height + 29) / 2, image=self.photo)
        else:
            self.photo = None
            self.create_text(width / 2, (height + 25) / 2, text=status,
                             fill='#c2d3e4', font=('TkDefaultFont', -17, 'bold'))
        self.create_text(10, 17, text=self.title, anchor='w', fill='#ffffff')
        detail = f'LIVE · {frame[3]:.1f} FPS · {frame[2][0]}×{frame[2][1]}' if frame else status
        self.create_text(width - 10, 17, text=detail, anchor='e',
                         fill={'LIVE': '#7aefc5', 'NO SIGNAL': '#ffd695'}.get(status, '#c2d3e4'))


class MonitorWindow:
    def __init__(self, root, node):
        self.root, self.node = root, node
        root.title('CORE MONITOR')
        root.geometry('1320x880')
        root.minsize(700, 570)
        root.configure(bg=BG)
        font.nametofont('TkDefaultFont', root=root).configure(size=-14)
        font.nametofont('TkTextFont', root=root).configure(size=-13)
        self.values = {}
        self.captions = {}
        self.history = {}
        self.latest_result = None
        self.pending = set()
        self.current_control = None
        self.graph_key = None
        self.layout_width = None

        header = tk.Frame(root, bg=NAVY, padx=20, pady=12)
        header.pack(fill='x')
        label(header, 'CORE MONITOR', bg=NAVY, fg='white', size=32, bold=True).pack(side='left')
        self.mode_label = label(header, node.mode, bg=TEAL if node.mode == 'FAKE' else BLUE, fg='white',
                                bold=True, padx=14, pady=4)
        self.mode_label.pack(side='right')

        viewport = tk.Frame(root, bg=BG)
        viewport.pack(fill='both', expand=True)
        self.scroller = tk.Canvas(viewport, bg=BG, highlightthickness=0)
        scroll = ttk.Scrollbar(viewport, command=self.scroller.yview)
        scroll.pack(side='right', fill='y')
        self.scroller.pack(side='left', fill='both', expand=True)
        self.scroller.configure(yscrollcommand=scroll.set)
        self.content = tk.Frame(self.scroller, bg=BG, padx=10, pady=8)
        self.canvas_window = self.scroller.create_window(0, 0, anchor='nw', window=self.content)
        self.content.bind('<Configure>', lambda _: self.scroller.configure(
            scrollregion=self.scroller.bbox('all')))
        self.scroller.bind('<Configure>', self.reflow)
        for button, direction in (('Button-4', -1), ('Button-5', 1)):
            root.bind(f'<{button}>', lambda event, d=direction: self.scroll_page(event, d))
        root.bind('<MouseWheel>', lambda event: self.scroll_page(event, -int(event.delta / 120)))

        self.controls = self.make_controls(root)
        self.controls.pack(side='bottom', fill='x', padx=10, pady=(0, 9))
        self.video = Card(self.content, '실시간 영상')
        views = tk.Frame(self.video.body, bg=CARD)
        views.pack(fill='both', expand=True)
        views.columnconfigure(0, weight=2)
        views.columnconfigure(1, weight=1)
        views.rowconfigure(0, weight=1)
        self.cameras = {}
        for column, (key, name) in enumerate((('wrist', '손목 카메라'), ('side', '측면 카메라'))):
            camera = CameraPanel(views, name)
            camera.grid(row=0, column=column, sticky='nsew', padx=(0, 5))
            self.cameras[key] = camera

        self.task = self.fields('현재 작업 상태', (
            ('FSM State', 'state'), ('Task ID', 'task'), ('작업 결과', 'outcome'),
            ('재가동 확인', 'resume')))
        self.devices = self.fields('장비 상태', tuple((key, key) for key in (
            'Control', 'Robot', 'Gripper', 'Camera', 'Vision', 'Voice', 'LLM', 'DB')), compact=True)
        self.inspection = self.fields('검사 및 체결 결과', (
            ('상단 검사', 'top'), ('측면 검사', 'side'), ('체결 판정', 'fastening'),
            ('최종 판정', 'final'), ('HOLD / FAIL 사유', 'reason'),
            ('최종 토크 / 회전', 'measurement'), ('레시피', 'recipe')))

        self.fsm = Card(self.content, '공정 진행')
        stages = tk.Frame(self.fsm.body, bg=CARD)
        stages.pack(fill='x')
        self.steps = {}
        for index, stage in enumerate(STAGES):
            stages.columnconfigure(index, weight=1, uniform='steps')
            chip = label(stages, bg=TRACK, size=11, bold=True, pady=6, justify='center')
            chip.grid(row=0, column=index, sticky='ew', padx=2)
            self.steps[stage] = chip

        self.tightening = Card(self.content, '체결 데이터')
        self.metrics = label(self.tightening.body, anchor='w', justify='left', wraplength=300)
        self.metrics.pack(fill='x', pady=(0, 3))
        self.metrics.bind('<Configure>', lambda event:
                          self.metrics.configure(wraplength=max(1, event.width - 4)))
        self.graph = tk.Canvas(self.tightening.body, bg='#f4f8fc', highlightthickness=0,
                               width=1, height=1)
        self.graph.pack(fill='both', expand=True)

        self.events = Card(self.content, '이벤트 로그')
        panel = tk.Frame(self.events.body, bg=CARD)
        panel.pack(fill='both', expand=True)
        scrollbar = ttk.Scrollbar(panel)
        scrollbar.pack(side='right', fill='y')
        self.journal = tk.Text(panel, bg='#f4f8fc', fg=NAVY, wrap='char', width=1, height=1,
                               relief='flat', padx=8, pady=5, state='disabled',
                               yscrollcommand=scrollbar.set)
        self.journal.pack(fill='both', expand=True)
        scrollbar.configure(command=self.journal.yview)
        for level, color in (('INFO', BLUE), ('WARN', AMBER), ('ERROR', RED)):
            self.journal.tag_configure(level, foreground=color)
        self.refresh_id = root.after(25, self.refresh)
        root.bind('<Destroy>', self.cancel_refresh, add='+')

    def cancel_refresh(self, event):
        if event.widget is self.root and self.refresh_id is not None:
            self.root.after_cancel(self.refresh_id)
            self.refresh_id = None

    def scroll_page(self, event, direction):
        if event.widget is not self.journal:
            self.scroller.yview_scroll(direction * 3, 'units')

    def fields(self, title, fields, *, compact=False):
        card = Card(self.content, title)
        for name, key in fields:
            row = tk.Frame(card.body, bg='#f2f6fa')
            row.pack(fill='x', pady=1 if compact else 2)
            caption = label(row, name, bg='#f2f6fa', fg=MUTED, size=13, padx=6)
            caption.pack(side='left')
            value = label(row, 'N/A', bg='#f2f6fa', bold=True, size=13,
                          padx=5, pady=2 if compact else 4, justify='right', anchor='e', wraplength=115)
            value.pack(side='right', fill='x', expand=True)
            row.bind('<Configure>', lambda event, widget=value, title=caption:
                     widget.configure(wraplength=max(1, event.width - title.winfo_reqwidth() - 14)))
            self.values[key] = value
            self.captions[key] = caption
        return card

    def make_controls(self, parent):
        card = Card(parent, '작업 제어')
        card.configure(height=111)
        buttons = tk.Frame(card.body, bg=CARD)
        buttons.pack(fill='x')
        self.buttons = {}
        for column, (command, color) in enumerate(zip(COMMANDS, (TEAL, BLUE, BLUE, MUTED, RED))):
            buttons.columnconfigure(column, weight=1, uniform='commands')
            button = tk.Canvas(buttons, width=1, height=32, bg=CARD, highlightthickness=1,
                               highlightbackground=CARD, highlightcolor=BLUE, takefocus=True)
            button.grid(row=0, column=column, sticky='ew', padx=3)
            button.bind('<ButtonRelease-1>', lambda event, cmd=command: self.send(cmd))
            for key in ('<Return>', '<space>'):
                button.bind(key, lambda event, cmd=command: self.send(cmd))
            self.buttons[command] = (button, color, None)
        label(card.body, 'STOP은 소프트웨어 정지 요청입니다. 물리적 비상정지 장치가 필요합니다.',
              fg=MUTED, size=12).pack(anchor='w', pady=(5, 0))
        return card

    def reflow(self, event):
        wide = event.width >= 1120
        self.scroller.itemconfigure(self.canvas_window, width=event.width,
                                    height=max(735, event.height) if wide else 0)
        if wide == self.layout_width:
            return
        self.layout_width = wide
        cards = (self.video, self.task, self.devices, self.fsm,
                 self.inspection, self.tightening, self.events)
        for card in cards:
            card.grid_forget()
        for col in range(12):
            self.content.columnconfigure(col, weight=1, uniform='dashboard')
        for row in (0, 2):
            self.content.rowconfigure(row, weight=1 if wide else 0)
        if wide:
            positions = ((0, 0, 6, 300), (0, 6, 3, 300), (0, 9, 3, 300),
                         (1, 0, 12, 104), (2, 0, 3, 291), (2, 3, 5, 291), (2, 8, 4, 291))
        else:
            positions = ((0, 0, 12, 260), (1, 0, 6, 226), (1, 6, 6, 300),
                         (2, 0, 12, 104), (3, 0, 5, 285), (3, 5, 7, 247), (4, 0, 12, 220))
        for card, (row, col, span, height) in zip(cards, positions):
            card.configure(height=height)
            card.grid(row=row, column=col, columnspan=span, sticky='nsew', padx=4, pady=4)

    def send(self, command):
        button, _, state = self.buttons[command]
        if command not in self.pending and state and state[0]:
            button.focus_set()
            self.pending.add(command)
            self.node.commands.put(command)
            self.render_buttons(self.current_control)

    def render_buttons(self, control=None):
        allowed = set(control.allowed_commands) if control else set()
        allowed.add('stop')  # Robot STOP remains available independently of Control.
        for command, (button, color, previous) in self.buttons.items():
            enabled = command in allowed and command not in self.pending
            width = button.winfo_width()
            state_key = (enabled, width)
            if state_key == previous:
                continue
            self.buttons[command] = (button, color, state_key)
            button.delete('all')
            if width > 10:
                rounded(button, 1, 1, width - 2, 30, 9, fill=color if enabled else '#dce5ee')
                button.create_text(width / 2, 16, text=COMMANDS[command],
                                   fill='white' if enabled else MUTED,
                                   font=('TkDefaultFont', -13, 'bold'))
            button.configure(cursor='hand2' if enabled else 'arrow')

    def record(self, task_id):
        if task_id not in self.history:
            if len(self.history) == 24:
                del self.history[next(iter(self.history))]
            self.history[task_id] = {'visited': {'READY'}, 'result': None}
        return self.history[task_id]

    def log(self, timestamp, level, node, stage, message, task_id=''):
        when = datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3]
        self.journal.configure(state='normal')
        line = f'{when}  [{level}]  {node} / {stage}'
        if task_id:
            line += f' / {task_id[:8]}'
        self.journal.insert('end', f'{line}\n  {message}\n', level)
        if int(self.journal.index('end-1c').split('.')[0]) > 602:
            self.journal.delete('1.0', '3.0')
        self.journal.configure(state='disabled')
        self.journal.see('end')

    def handle(self, kind, payload):
        if kind == 'command_done':
            self.pending.discard(payload)
            return
        if kind == 'command':
            command, target, accepted, reason = payload
            self.log(datetime.now().timestamp(), 'INFO' if accepted else 'WARN',
                     target, COMMANDS[command], reason)
            return
        task_id = payload.task_id
        record = self.record(task_id) if task_id else None
        when = payload.stamp.sec + payload.stamp.nanosec / 1e9
        if kind == 'state':
            return
        if kind == 'result':
            if record is None:
                return
            record['result'] = payload
            self.latest_result = task_id
            self.log(when, {'PASS': 'INFO', 'HOLD': 'WARN', 'FAIL': 'ERROR'}.get(
                payload.outcome, 'INFO'), 'control', 'RESULT',
                f'{payload.outcome} · {payload.reason}', task_id)
            return
        self.log(when, payload.log_level.upper(), payload.node or 'unknown',
                 payload.stage, f'[{payload.source}] {payload.log_msg}' if payload.source
                 else payload.log_msg, task_id)

    def set_value(self, key, value, color=None):
        self.values[key].configure(text=value, fg=color or COLORS.get(value, NAVY))

    def render_progress(self, record, state):
        result = record.get('result')
        visited = set(result.states) if result else record.get('visited', set())
        failed = next((step for step in reversed(STAGES[:-1]) if step in visited), None)
        for stage, label_widget in self.steps.items():
            name, color, background = '대기', MUTED, TRACK
            if result and result.outcome in ('FAIL', 'HOLD') and stage == failed:
                if result.outcome == 'FAIL':
                    name, color, background = '실패', RED, '#fae7e9'
                else:
                    name, color, background = '보류', AMBER, '#fff2d9'
            elif stage == state:
                name, color, background = '현재', 'white', TEAL
            elif stage in visited:
                name, color, background = '완료', GREEN, '#dff4ed'
            elif stage in OPTIONAL and visited & AFTER_BRANCH and not visited & OPTIONAL:
                name, color, background = '생략', MUTED, '#f1f4f8'
            label_widget.configure(text=f'{stage}\n{name}', fg=color, bg=background)

    def render_graph(self, data):
        width, height = self.graph.winfo_width(), self.graph.winfo_height()
        key = (width, height, data['task'], data['last'], self.display_mode) if data else (width, height)
        if self.graph_key == key:
            return
        self.graph_key = key
        self.graph.delete('all')
        self.metrics.configure(text='최대 토크  N/A   ·   최대 축력  N/A\n체결 시간  N/A')
        if not data:
            self.graph.create_text(width / 2, height / 2, text='N/A · 체결 데이터 수신 대기', fill=MUTED)
            return
        elapsed = (data['last'] - data['first']) / 1e9
        source = 'MOCK · ' if self.display_mode == 'FAKE' else ''
        duration = f'{elapsed:.2f} s' if elapsed > 0 else 'N/A'
        self.metrics.configure(text=f"{source}최대 토크  {data['torque']:.2f} N·m   ·   "
                                    f"최대 축력  {data['force']:.2f} N\n체결 시간 (샘플 구간)  {duration}")
        if width < 100 or height < 85:
            return
        samples = data['samples']
        start = samples[0][0]
        span = max(samples[-1][0] - start, 0.001)
        left, right = 8, width - 8
        for index, (field, title, color) in enumerate((
                ('force', '축력 Fz (N)', TEAL), ('torque', '모멘트 Mz (N·m)', BLUE)), 1):
            band = (height - 21) / 2
            top = (index - 1) * band + 17
            mid = top + (band - 17) / 2
            limit = max(data[field], 0.001)
            self.graph.create_text(left, top - 8, anchor='w',
                                   text=f'{title}   ±{limit:.2f}', fill=color)
            self.graph.create_line(left, mid, right, mid, fill='#bacddd', dash=(2, 4))
            coords = []
            for sample in samples:
                coords.extend((left + (sample[0] - start) / span * (right - left),
                               mid - sample[index] / limit * (band - 17) / 2))
            if len(coords) >= 4:
                self.graph.create_line(*coords, fill=color, width=2)
            else:
                x, y = coords
                self.graph.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline='')
        self.graph.create_text(left, height - 8, text=f'{start:.2f} s', anchor='w', fill=MUTED)
        self.graph.create_text(right, height - 8, text=f'{samples[-1][0]:.2f} s',
                               anchor='e', fill=MUTED)

    def refresh(self):
        self.refresh_id = None
        if self.node.closed.is_set():
            self.root.destroy()
            return
        events, control_sample, robot_sample, equipment, frames, data = self.node.snapshot()
        now = monotonic()
        def live(sample):
            if sample and now - sample[1] < self.node.status_timeout:
                if current_stamp(self.node, sample[0].stamp, self.node.status_timeout):
                    return sample[0]
            return None

        control, robot = live(control_sample), live(robot_sample)
        self.current_control = control
        for kind, event in events:
            self.handle(kind, event)
        state = control.state if control else 'NO SIGNAL'
        active = control.task_id if control else ''
        self.display_mode = control.mode.upper() if control else self.node.mode
        self.mode_label.configure(text=self.display_mode if control else f'{self.display_mode} · 대기')
        displayed_task = active or (control.last_task_id if control else self.latest_result)
        historical = self.history.get(displayed_task, {})
        result = historical.get('result')
        progress = {'visited': set(control.states)} if control else {}
        if result:
            progress['result'] = result
        outcome = control.outcome if control else (result.outcome if result else '')
        reason = ((control.stop_reason or control.last_reason) if control else
                  (result.reason if result else ''))
        measurements = 'N/A'
        if result and result.tightening_valid:
            prefix = 'MOCK · ' if result.mode == 'fake' else ''
            measurements = f'{prefix}{result.peak_torque:.2f} N·m / {result.turns:.2f}'
        record = control or result
        inspection_only = self.display_mode == 'INSPECTION'
        inspection_values = [getattr(record, key, '') or 'N/A'
                             for key in ('top', 'side', 'fastening')]
        if inspection_only:
            inspection_values = [
                f'{getattr(record, "presence_" + point, "") or "N/A"} / '
                f'{getattr(record, "fastening_" + point, "") or "N/A"}'
                for point in ('l', 'r')
            ]
            inspection_values.append(
                f'{getattr(record, "presence_evidence", "") or "UNVERIFIED"} / '
                f'{getattr(record, "fastening_evidence", "") or "UNVERIFIED"}')
        captions = (('Left(L) 존재 / 체결', 'Right(R) 존재 / 체결', '판정 근거') if inspection_only
                    else ('상단 검사', '측면 검사', '체결 판정'))
        for key, caption, value in zip(('top', 'side', 'fastening'), captions, inspection_values):
            self.captions[key].configure(text=caption)
            self.set_value(key, value)
        for key, value in (
            ('state', state), ('task', displayed_task or 'N/A'),
            ('outcome', outcome or ('RUNNING' if active else 'N/A')),
            ('resume', control.recovery or '—' if control else '—'),
            ('final', outcome or 'N/A'),
            ('reason', reason if state == 'STOPPED' or outcome in ('HOLD', 'FAIL') else '—'),
            ('measurement', measurements), ('recipe', result.recipe_id if result and result.recipe_id else 'N/A'),
            ('Control', 'ONLINE' if control else 'NO SIGNAL'),
            ('Vision', control.vision_status or 'UNVERIFIED' if control else 'UNVERIFIED'),
            ('Robot', 'NO SIGNAL' if not robot else
             ('STOP CONFIRMED' if robot.stop_confirmed else 'STOP REQUESTED') if robot.stopped
             else 'BUSY' if robot.busy else 'READY' if robot.safe_to_move else 'BLOCKED'),
            ('Gripper', 'N/A' if not robot or not robot.gripper_valid else
             f'{robot.gripper_width * 1000:.1f} mm · ' +
             (robot.holding_item or 'GRIP' if robot.grip_detected else 'OPEN')),
        ):
            self.set_value(key, value)
        for name, value in equipment.items():
            self.set_value(name, value, MUTED)
        camera_statuses = []
        for key, widget in self.cameras.items():
            enabled = bool(self.node.cameras[key])
            frame = frames.get(key) if enabled else None
            if frame and now - frame[1] >= self.node.signal_timeout:
                frame = None
            signal = 'LIVE' if frame else 'NO SIGNAL' if enabled else 'OFF'
            if enabled:
                camera_statuses.append(signal)
            widget.render(signal, frame)
        camera = ('LIVE' if camera_statuses and all(s == 'LIVE' for s in camera_statuses)
                  else 'PARTIAL' if 'LIVE' in camera_statuses
                  else 'NO SIGNAL' if camera_statuses else 'OFF')
        self.set_value('Camera', camera, GREEN if camera == 'LIVE' else MUTED)
        self.render_progress(progress, state)
        self.render_graph(data if data and data['task'] == displayed_task else None)
        self.render_buttons(control)
        self.refresh_id = self.root.after(60, self.refresh)


def main():
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise SystemExit(f'CORE MONITOR requires a graphical display: {exc}') from exc
    with ros_context():
        node = MonitorNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def spin():
            try:
                executor.spin()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                node.closed.set()

        ros_thread = Thread(target=spin, name='monitor-ros')
        camera_thread = Thread(target=node.decode_images, name='monitor-camera')

        def close_monitor(*_):
            node.closed.set()

        handlers = {kind: signal.signal(kind, close_monitor) for kind in (signal.SIGINT, signal.SIGTERM)}
        try:
            MonitorWindow(root, node)
            ros_thread.start()
            camera_thread.start()
            root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            for kind, handler in handlers.items():
                signal.signal(kind, handler)
            node.closed.set()
            node.image_ready.set()
            executor.shutdown()
            for thread in (ros_thread, camera_thread):
                if thread.ident is not None:
                    thread.join()
            node.destroy_node()
