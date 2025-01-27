import sqlite3

database = "././pib_api/flask/pibdata.db"


def ask_for_bricklet_uids(bricklet_numbers: list[int]) -> list[str]:
    new_uids = []
    for bricklet_number in bricklet_numbers:
        while True:
            user_uid = input(f"Please enter the ID for bricklet {bricklet_number}: ")
            if not user_uid or user_uid.isalnum():
                new_uids.append(user_uid)
                break
            print("Only letters and numbers are allowed")
    return new_uids


def get_bricklet_numbers_from_db() -> list[int]:
    try:
        with sqlite3.connect(database) as conn:
            cur = conn.cursor()
            bricklet_numbers = cur.execute(
                "SELECT bricklet_number FROM bricklet"
            ).fetchall()

        return [bricklet_number[0] for bricklet_number in bricklet_numbers]
    except sqlite3.OperationalError as e:
        print(e)
        print("Could not get bricklet_numbers from database")


def update_bricklet_uids(new_uids: list[str], bricklet_numbers: list[int]) -> None:
    try:
        with sqlite3.connect(database) as conn:
            cur = conn.cursor()

            assert len(new_uids) == len(bricklet_numbers)
            for uid, bricklet_number in zip(new_uids, bricklet_numbers):
                if not uid:
                    continue
                cur.execute(
                    "UPDATE bricklet SET uid = ? WHERE (bricklet_number) = ?",
                    (f"uid{bricklet_number}", bricklet_number),
                )

            for uid, bricklet_number in zip(new_uids, bricklet_numbers):
                if not uid:
                    continue
                cur.execute(
                    "UPDATE bricklet SET uid = ? WHERE (bricklet_number) = ?",
                    (uid, bricklet_number),
                )

            print("New uids were successfully set")
    except (sqlite3.OperationalError, sqlite3.IntegrityError) as e:
        print(e)
        print("Could not update bricklet-uids")


if __name__ == "__main__":
    bricklet_numbers = get_bricklet_numbers_from_db()
    new_uids = ask_for_bricklet_uids(bricklet_numbers)
    update_bricklet_uids(new_uids, bricklet_numbers)
