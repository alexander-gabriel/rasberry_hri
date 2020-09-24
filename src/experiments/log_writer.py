import sqlite3


class DB:
    def __init__(self, filename='/home/rasberry/log.db'):
        self.db = sqlite3.connect(filename)
        try:
            self.build_db()
        except sqlite3.OperationalError:
            pass

    def build_db(self):
        cursor = self.db.cursor()
        cursor.execute("CREATE TABLE movement_variance (timestamp float, x float, y float, typ text, target text)")
        self.db.commit()

    def add_entry(self, timestamp, typ, target, x, y):
        cursor = self.db.cursor()
        cursor.execute("INSERT INTO movement_variance VALUES (?, ?, ?, ?, ?)", (timestamp, x, y, typ, target))
        self.db.commit()

    def get_entries(self):
        cursor = self.db.cursor()
        cursor.execute("SELECT * from movement_variance")
        return cursor.fetchall()

    def close_db(self):
        self.db.close()
