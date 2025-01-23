import sqlite3

database = "./pib_api/flask/pibdata.db"
bricklet_numbers = [1, 2, 3]


def ask_for_bricklet_uids() -> list[str]:
    new_uids = []
    for bricklet_number in bricklet_numbers:
        user_uid = input(f"Please enter the ID for bricklet {bricklet_number}: ")
        if user_uid and not user_uid.isalnum():
            raise ValueError("Only numbers and letters are allowed")
        new_uids.append(user_uid)
    return new_uids


def update_bricklet_uids() -> None:
    try:
        with sqlite3.connect(database) as conn:
            cur = conn.cursor()

            new_uids = ask_for_bricklet_uids()

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
    except Exception as e:
        print(e)
        print("New uids could not be set")


if __name__ == "__main__":
    update_bricklet_uids()
