"""add assistant

Revision ID: 330f7919a6f3
Revises: ce188a77a5c2
Create Date: 2024-04-09 08:47:06.670369

"""
from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = '330f7919a6f3'
down_revision = 'ce188a77a5c2'
branch_labels = None
depends_on = None


def upgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.create_table('assistant_model',
    sa.Column('id', sa.Integer(), nullable=False),
    sa.Column('api_name', sa.String(length=255), nullable=False),
    sa.Column('visual_name', sa.String(length=255), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.UniqueConstraint('api_name'),
    sa.UniqueConstraint('visual_name')
    )
    with op.batch_alter_table('personality', schema=None) as batch_op:
        batch_op.add_column(sa.Column('assistant_id', sa.Integer(), nullable=True))
        batch_op.create_foreign_key("fk_assistant_id", 'assistant_model', ['assistant_id'], ['id'])

    # ### end Alembic commands ###


def downgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    with op.batch_alter_table('personality', schema=None) as batch_op:
        batch_op.drop_constraint("fk_assistant_id", type_='foreignkey')
        batch_op.drop_column('assistant_id')

    op.drop_table('assistant_model')
    # ### end Alembic commands ###