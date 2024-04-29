"""init

Revision ID: 7a4053561c44
Revises: 
Create Date: 2024-04-29 08:32:44.617874

"""
from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = '7a4053561c44'
down_revision = None
branch_labels = None
depends_on = None


def upgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.create_table('assistant_model',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('api_name', sa.String(length=255), nullable=False),
    sa.Column('visual_name', sa.String(length=255), nullable=False),
    sa.Column('has_image_support', sa.Boolean(), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('api_name'),
    sa.UniqueConstraint('visual_name')
    )
    op.create_table('bricklet',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('uid', sa.String(length=30), nullable=False),
    sa.Column('bricklet_number', sa.Integer(), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('bricklet_number'),
    sa.UniqueConstraint('uid')
    )
    op.create_table('cameraSettings',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('resolution', sa.String(length=3), nullable=False),
    sa.Column('refresh_rate', sa.Float(), nullable=False),
    sa.Column('quality_factor', sa.Integer(), nullable=False),
    sa.Column('res_x', sa.Integer(), nullable=False),
    sa.Column('res_y', sa.Integer(), nullable=False),
    sa.PrimaryKeyConstraint('id')
    )
    op.create_table('motor',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('name', sa.String(length=255), nullable=False),
    sa.Column('pulse_width_min', sa.Integer(), nullable=False),
    sa.Column('pulse_width_max', sa.Integer(), nullable=False),
    sa.Column('rotation_range_min', sa.Integer(), nullable=False),
    sa.Column('rotation_range_max', sa.Integer(), nullable=False),
    sa.Column('velocity', sa.Integer(), nullable=False),
    sa.Column('acceleration', sa.Integer(), nullable=False),
    sa.Column('deceleration', sa.Integer(), nullable=False),
    sa.Column('period', sa.Integer(), nullable=False),
    sa.Column('turned_on', sa.Boolean(), nullable=False),
    sa.Column('visible', sa.Boolean(), nullable=False),
    sa.Column('invert', sa.Boolean(), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('name')
    )
    op.create_table('program',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('name', sa.String(length=255), nullable=False),
    sa.Column('code_visual', sa.String(length=100000), nullable=False),
    sa.Column('program_number', sa.String(length=50), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('name'),
    sa.UniqueConstraint('program_number')
    )
    op.create_table('brickletPin',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('motor_id', sa.Integer(), nullable=False),
    sa.Column('bricklet_id', sa.Integer(), nullable=False),
    sa.Column('pin', sa.Integer(), nullable=False),
    sa.Column('invert', sa.Boolean(), nullable=False),
    sa.ForeignKeyConstraint(['bricklet_id'], ['bricklet.id'], ),
    sa.ForeignKeyConstraint(['motor_id'], ['motor.id'], ),
    sa.PrimaryKeyConstraint('id')
    )
    op.create_table('personality',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('name', sa.String(length=255), nullable=False),
    sa.Column('personality_id', sa.String(length=255), nullable=False),
    sa.Column('gender', sa.String(length=255), nullable=False),
    sa.Column('description', sa.String(length=38000), nullable=True),
    sa.Column('pause_threshold', sa.Float(), nullable=False),
    sa.Column('assistant_model_id', sa.Integer(), nullable=False),
    sa.ForeignKeyConstraint(['assistant_model_id'], ['assistant_model.id'], ),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('personality_id')
    )
    op.create_table('chat',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('chat_id', sa.String(length=255), nullable=False),
    sa.Column('topic', sa.String(length=255), nullable=False),
    sa.Column('personality_id', sa.String(length=255), nullable=False),
    sa.ForeignKeyConstraint(['personality_id'], ['personality.personality_id'], ),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('chat_id')
    )
    op.create_table('chatMessage',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('message_id', sa.String(length=255), nullable=False),
    sa.Column('timestamp', sa.DateTime(), nullable=False),
    sa.Column('is_user', sa.Boolean(), nullable=False),
    sa.Column('content', sa.String(length=100000), nullable=False),
    sa.Column('chat_id', sa.String(length=255), nullable=False),
    sa.ForeignKeyConstraint(['chat_id'], ['chat.chat_id'], ),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('message_id')
    )
    # ### end Alembic commands ###


def downgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.drop_table('chatMessage')
    op.drop_table('chat')
    op.drop_table('personality')
    op.drop_table('brickletPin')
    op.drop_table('program')
    op.drop_table('motor')
    op.drop_table('cameraSettings')
    op.drop_table('bricklet')
    op.drop_table('assistant_model')
    # ### end Alembic commands ###
