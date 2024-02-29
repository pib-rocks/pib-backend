"""init

Revision ID: ce188a77a5c2
Revises: 
Create Date: 2024-02-20 11:24:32.387577

"""
from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = 'ce188a77a5c2'
down_revision = None
branch_labels = None
depends_on = None


def upgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.create_table('bricklet',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('uid', sa.String(length=30), nullable=False),
    sa.Column('brickletNumber', sa.Integer(), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('brickletNumber'),
    sa.UniqueConstraint('uid')
    )
    op.create_table('cameraSettings',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('resolution', sa.String(length=3), nullable=False),
    sa.Column('refreshRate', sa.Float(), nullable=False),
    sa.Column('qualityFactor', sa.Integer(), nullable=False),
    sa.Column('resX', sa.Integer(), nullable=False),
    sa.Column('resY', sa.Integer(), nullable=False),
    sa.PrimaryKeyConstraint('id')
    )
    op.create_table('motor',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('name', sa.String(length=255), nullable=False),
    sa.Column('pulseWidthMin', sa.Integer(), nullable=False),
    sa.Column('pulseWidthMax', sa.Integer(), nullable=False),
    sa.Column('rotationRangeMin', sa.Integer(), nullable=False),
    sa.Column('rotationRangeMax', sa.Integer(), nullable=False),
    sa.Column('velocity', sa.Integer(), nullable=False),
    sa.Column('acceleration', sa.Integer(), nullable=False),
    sa.Column('deceleration', sa.Integer(), nullable=False),
    sa.Column('period', sa.Integer(), nullable=False),
    sa.Column('turnedOn', sa.Boolean(), nullable=False),
    sa.Column('visible', sa.Boolean(), nullable=False),
    sa.Column('invert', sa.Boolean(), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('name')
    )
    op.create_table('personality',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('name', sa.String(length=255), nullable=False),
    sa.Column('personalityId', sa.String(length=255), nullable=False),
    sa.Column('gender', sa.String(length=255), nullable=False),
    sa.Column('description', sa.String(length=38000), nullable=True),
    sa.Column('pauseThreshold', sa.Float(), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('personalityId')
    )
    op.create_table('program',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('name', sa.String(length=255), nullable=False),
    sa.Column('codeVisual', sa.String(length=100000), nullable=False),
    sa.Column('programNumber', sa.String(length=50), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('name')
    )
    op.create_table('brickletPin',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('motorId', sa.Integer(), nullable=False),
    sa.Column('brickletId', sa.Integer(), nullable=False),
    sa.Column('pin', sa.Integer(), nullable=False),
    sa.Column('invert', sa.Boolean(), nullable=False),
    sa.ForeignKeyConstraint(['brickletId'], ['bricklet.id'], ),
    sa.ForeignKeyConstraint(['motorId'], ['motor.id'], ),
    sa.PrimaryKeyConstraint('id')
    )
    op.create_table('chat',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('chatId', sa.String(length=255), nullable=False),
    sa.Column('topic', sa.String(length=255), nullable=False),
    sa.Column('personalityId', sa.String(length=255), nullable=False),
    sa.ForeignKeyConstraint(['personalityId'], ['personality.personalityId'], ),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('chatId')
    )
    op.create_table('chatMessage',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('messageId', sa.String(length=255), nullable=False),
    sa.Column('timestamp', sa.DateTime(), nullable=False),
    sa.Column('isUser', sa.Boolean(), nullable=False),
    sa.Column('content', sa.String(length=100000), nullable=False),
    sa.Column('chatId', sa.String(length=255), nullable=False),
    sa.ForeignKeyConstraint(['chatId'], ['chat.chatId'], ),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('messageId')
    )
    # ### end Alembic commands ###


def downgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.drop_table('chatMessage')
    op.drop_table('chat')
    op.drop_table('brickletPin')
    op.drop_table('program')
    op.drop_table('personality')
    op.drop_table('motor')
    op.drop_table('cameraSettings')
    op.drop_table('bricklet')
    # ### end Alembic commands ###
