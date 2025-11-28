import sqlite3
import sys

db_path = sys.argv[1] if len(sys.argv) > 1 else "data/task_history.db"

conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# List all tables
cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
tables = cursor.fetchall()
print(f"Database: {db_path}")
print(f"Tables: {[t[0] for t in tables]}\n")

# Show contents of each table
for (table,) in tables:
    cursor.execute(f"SELECT * FROM {table} LIMIT 10")
    rows = cursor.fetchall()
    print(f"--- {table} ({len(rows)} rows shown) ---")
    for row in rows:
        print(row)
    print()

conn.close()
