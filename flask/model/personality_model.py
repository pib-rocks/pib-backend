from app.app import db
import uuid

class Personality(db.Model):

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False)
    personalityId = db.Column(db.String(255), nullable=False, unique=True)
    gender = db.Column(db.String(255), nullable=False)
    description = db.Column(db.String(38000), nullable=True)
    pauseThreshold = db.Column(db.Float, nullable=False)
    
    def __init__(self, *args):
        if len(args) == 3:
            self.eq3(args)
        if len(args) == 5:
            self.eq5(args)

    def eq3(self, args):
        self.name = args[0]
        self.personalityId = str(uuid.uuid4())
        self.description = ""
        self.gender = args[1]
        self.pauseThreshold = args[2]

    def eq5(self, args):
        self.name = args[0]
        self.personalityId = args[1]
        self.gender = args[2]
        self.description = args[3]
        self.pauseThreshold = args[4]