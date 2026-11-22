"""ジョブログを読み、エラーとその診断に必要な行を時系列で抜粋する。"""
from collections import deque
import re


ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
ERROR = re.compile(
    r'\[(?:ERROR|FATAL|CRITICAL)\]|\b(?:error|fatal):|\b(?:failed|segmentation fault|out of memory)\b'
    r'|\b[\w.]*(?:Error|Exception):|\[ros2run\]: Process exited with failure'
    r'|エラー[:：]|失敗しました|タイムアウトしました', re.I)
WARNING = re.compile(r'\[WARN(?:ING)?\]|\bwarning:', re.I)
CAPTURE_DIAGNOSTIC = re.compile(r'Saved-map localized: (?:True|False)|Successfully localized at|VGL pose hint received'
                                r'|bag再生前の接続待ち|bag内の画像stamp|vo_status[:=]|点群の更新時刻', re.I)
FRAME_GAP = re.compile(r'Delta between current and previous frame\s*\[(\d+(?:\.\d+)?)\s*ms\]', re.I)
ROS_PROCESS = re.compile(r'^\[[\w.-]+-\d+\] ?')


def error_excerpt(lines, failure_message=''):
    first, last = [], deque(maxlen=80)
    warnings, traces, capture = deque(maxlen=5), deque(maxlen=80), {}
    max_gap_ms = 0.
    previous = None
    traceback = False
    following = 0
    matched = False
    for number, raw in enumerate(lines, 1):
        line = ANSI.sub('', raw).rstrip('\r\n')
        if len(line) > 2048:
            line = line[:1024] + ' …（長い行を省略）… ' + line[-1024:]
        item = (number, line)
        content = ROS_PROCESS.sub('', line)
        start = 'Traceback (most recent call last)' in content
        marked = start or bool(ERROR.search(content))
        if start or traceback:
            traces.append(item)
        diagnostic = CAPTURE_DIAGNOSTIC.search(content)
        if diagnostic:
            capture[diagnostic.group().casefold()] = item
        gap = FRAME_GAP.search(content)
        if gap and float(gap[1]) > max_gap_ms:
            max_gap_ms = float(gap[1])
            capture['max_frame_gap'] = item
        if WARNING.search(content):
            warnings.append(item)
        if marked or traceback or following:
            if marked and not traceback and not following and previous:
                (first if len(first) < 40 else last).append(previous)
            (first if len(first) < 40 else last).append(item)
        if marked:
            matched = True
            following = 2
        elif following:
            following -= 1
        # Python prints indented frames followed by the unindented final exception.
        traceback = start or (traceback and (not content.strip() or content.startswith((' ', '\t'))))
        previous = item

    if not matched and not failure_message:
        return ''
    entries = sorted(dict([*first, *last, *traces, *warnings, *capture.values()]).items())
    output = []
    last_number = 0
    for number, line in entries:
        if last_number and number > last_number + 1:
            output.append('…')
        output.append(line)
        last_number = number
    if failure_message and not any(failure_message in line for _, line in entries):
        output.append('ジョブの終了理由: ' + failure_message)
    return '\n'.join(output).strip()
