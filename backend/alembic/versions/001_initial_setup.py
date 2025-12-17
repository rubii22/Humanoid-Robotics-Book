"""Initial migration

Revision ID: 001_initial_setup
Revises: 
Create Date: 2025-12-17

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = '001_initial_setup'
down_revision = None
branch_labels = None
depends_on = None


def upgrade():
    # Create book_content table
    op.create_table('book_content',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('title', sa.String(length=500), nullable=False),
        sa.Column('author', sa.String(length=200), nullable=False),
        sa.Column('isbn', sa.String(length=20), nullable=True),
        sa.Column('publication_date', sa.DateTime(), nullable=True),
        sa.Column('content_type', sa.String(length=50), nullable=False, server_default='technical_book'),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.Column('updated_at', sa.DateTime(), nullable=False),
        sa.PrimaryKeyConstraint('id')
    )

    # Create text_chunks table
    op.create_table('text_chunks',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('book_content_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('chapter', sa.String(length=100), nullable=False),
        sa.Column('section', sa.String(length=100), nullable=False),
        sa.Column('paragraph_number', sa.Integer(), nullable=True),
        sa.Column('page_number', sa.Integer(), nullable=True),
        sa.Column('position_in_book', sa.Integer(), nullable=False),
        sa.Column('metadata_json', sa.Text(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.Column('updated_at', sa.DateTime(), nullable=False),
        sa.ForeignKeyConstraint(['book_content_id'], ['book_content.id'], ),
        sa.PrimaryKeyConstraint('id')
    )

    # Create query_logs table
    op.create_table('query_logs',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('query_text', sa.Text(), nullable=False),
        sa.Column('query_type', sa.String(length=50), nullable=False),
        sa.Column('selected_text', sa.Text(), nullable=True),
        sa.Column('retrieved_chunks', postgresql.JSONB(astext_type=sa.Text()), nullable=True),
        sa.Column('response', sa.Text(), nullable=False),
        sa.Column('was_refusal', sa.Boolean(), nullable=False, server_default='false'),
        sa.Column('confidence_score', sa.Float(), nullable=True),
        sa.Column('request_metadata', postgresql.JSONB(astext_type=sa.Text()), nullable=True),
        sa.Column('book_content_id', postgresql.UUID(as_uuid=True), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.Column('updated_at', sa.DateTime(), nullable=False),
        sa.ForeignKeyConstraint(['book_content_id'], ['book_content.id'], ),
        sa.PrimaryKeyConstraint('id')
    )

    # Create indexes
    op.create_index('ix_text_chunks_book_content_id', 'text_chunks', ['book_content_id'])
    op.create_index('ix_query_logs_book_content_id', 'query_logs', ['book_content_id'])
    op.create_index('ix_query_logs_query_type', 'query_logs', ['query_type'])


def downgrade():
    op.drop_index('ix_query_logs_query_type', table_name='query_logs')
    op.drop_index('ix_query_logs_book_content_id', table_name='query_logs')
    op.drop_index('ix_text_chunks_book_content_id', table_name='text_chunks')
    
    op.drop_table('query_logs')
    op.drop_table('text_chunks')
    op.drop_table('book_content')