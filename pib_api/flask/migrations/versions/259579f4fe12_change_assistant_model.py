"""change_assistant_model

Revision ID: 259579f4fe12
Revises: 7a4053561c44
Create Date: 2024-05-28 08:43:54.207287

"""
from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = '259579f4fe12'
down_revision = '7a4053561c44'
branch_labels = None
depends_on = None


def upgrade():
    op.create_table('assistant_model_new',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('api_name', sa.String(length=255), nullable=False),
    sa.Column('visual_name', sa.String(length=255), nullable=False),
    sa.Column('has_image_support', sa.Boolean(), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('visual_name')
    )
    conn = op.get_bind()
    
    conn.execute(
        sa.text("""
            INSERT INTO assistant_model_new
            SELECT * FROM assistant_model;
             """)
    )
    op.drop_table('assistant_model')
    op.rename_table('assistant_model_new', 'assistant_model')
    conn.execute(
        sa.text("""
            UPDATE assistant_model
            SET api_name = 'gpt-4o', visual_name = 'GPT-4o (with image)'
            WHERE api_name = 'gpt-4-turbo';
            """)
    )
    conn.execute(
    sa.text("""
        INSERT OR IGNORE INTO assistant_model (api_name, visual_name, has_image_support)
        VALUES ('gpt-4o', 'GPT-4o (without image)', false)
        """)
    )



def downgrade():
    op.create_table('assistant_model_new',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('api_name', sa.String(length=255), nullable=False),
    sa.Column('visual_name', sa.String(length=255), nullable=False),
    sa.Column('has_image_support', sa.Boolean(), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('api_name'),
    sa.UniqueConstraint('visual_name')
    )
    conn = op.get_bind()
    conn.execute(
        sa.text("""
            INSERT OR IGNORE INTO assistant_model_new
            SELECT * FROM assistant_model;
             """)
    )
    op.drop_table('assistant_model')
    op.rename_table('assistant_model_new', 'assistant_model')

