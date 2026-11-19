"""選択した速度 FF 表を原子的に保存し、外部からの配置・置換・削除にも追従する。"""
import json
import os
from pathlib import Path
import tempfile

from .speed import FeedforwardTable


class SpeedTableFile:
    def __init__(self, path):
        self.path = Path(path)
        self.revision = None
        self.table = None
        self.issue = ''

    def refresh(self):
        try:
            stat = self.path.stat()
            revision = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
        except OSError as exc:
            revision = ('unavailable', exc.errno)
        if self.revision == revision:
            return False
        self.revision = revision
        self.table = None
        try:
            with self.path.open(encoding='utf-8') as stream:
                self.table = FeedforwardTable.from_document(json.load(stream))
            self.issue = ''
        except FileNotFoundError:
            self.issue = '速度 FF 表が未設定です。Notebook の車両校正から表を選んでください'
        except (OSError, ValueError, RecursionError) as exc:
            self.issue = f'速度 FF 表を読み込めません（{exc}）'
        return True

    def save(self, doc):
        selected = FeedforwardTable.from_document(doc)
        if self.table == selected:
            try:
                with self.path.open(encoding='utf-8') as stream:
                    if json.load(stream) == doc:
                        return selected
            except (OSError, ValueError, RecursionError):
                pass
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent, delete=False) as out:
                temporary = Path(out.name)
                json.dump(doc, out, ensure_ascii=False, allow_nan=False)
                out.write('\n')
                out.flush()
                os.fsync(out.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        self.revision = None
        return selected
