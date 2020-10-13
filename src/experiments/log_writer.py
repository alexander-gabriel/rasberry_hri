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
        cursor.execute("CREATE TABLE picker_behavior (timestamp float, x float, y float, orientation float, behaviour text)")
        cursor.execute("CREATE TABLE picker_waiting (timestamp float, x float, y float, orientation float, behaviour text, wait float)")
        cursor.execute("CREATE TABLE robot_behavior (timestamp float, x float, y float, orientation float, behaviour text)")
        self.db.commit()

    def add_entry(self, timestamp, typ, target, x, y):
        cursor = self.db.cursor()
        cursor.execute("INSERT INTO movement_variance VALUES (?, ?, ?, ?, ?)", (timestamp, x, y, typ, target))
        self.db.commit()

    def add_person_entry(self, timestamp, x, y, orientation, behaviour):
        cursor = self.db.cursor()
        cursor.execute("INSERT INTO picker_behavior VALUES (?, ?, ?, ?, ?)",
                       (timestamp, x, y, orientation, behaviour))
        self.db.commit()

    def add_person_wait_entry(self, timestamp, x, y,
                              orientation, behaviour, wait):
        cursor = self.db.cursor()
        cursor.execute("INSERT INTO picker_waiting VALUES (?, ?, ?, ?, ?, ?)",
                       (timestamp, x, y, orientation, behaviour, wait))
        self.db.commit()

    def add_robot_entry(self, timestamp, x, y, orientation, behaviour):
        cursor = self.db.cursor()
        cursor.execute("INSERT INTO robot_behavior VALUES (?, ?, ?, ?, ?)",
                       (timestamp, x, y, orientation, behaviour))
        self.db.commit()

    def get_entries(self):
        cursor = self.db.cursor()
        cursor.execute("SELECT * from movement_variance")
        return cursor.fetchall()

    def close_db(self):
        self.db.close()
