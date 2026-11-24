"""Alembic 마이그레이션 실행 환경: `alembic upgrade`가 이 파일을 실행해 작업 DB 스키마를 최신으로 올린다.

관제 서버도 시작할 때 같은 경로를 탄다(storage/store.py의 TaskStore._migrate가 `alembic upgrade head`를 부른다).
DB 주소는 alembic.ini의 `sqlalchemy.url`이고, TaskStore는 그 값을 실제 DB 파일 경로로 바꿔 넣는다.

- 비교 대상 메타데이터는 control_station.storage.database.Base.metadata다(ORM 모델 정의).
- 오프라인(SQL만 출력)·온라인(실제 연결) 모두 `render_as_batch=True`로 돈다. SQLite는 ALTER TABLE이 제한적이라
  열을 바꾸거나 지울 때 "새 테이블을 만들어 복사하고 바꿔 끼우는" batch 방식이 필요하다.
- 온라인 모드는 연결마다 `PRAGMA foreign_keys=ON`을 켠다. SQLite는 연결마다 외래 키 검사가 꺼진 채로 시작하므로,
  켜지 않으면 ON DELETE CASCADE 같은 제약이 마이그레이션 중에 지켜지지 않는다(database.create_database_engine과 같은 이유).
"""

from __future__ import annotations

from logging.config import fileConfig

from alembic import context
from sqlalchemy import engine_from_config, event, pool

from control_station.storage.database import Base


# alembic.ini의 내용. 파일로 실행했으면 그 안의 [loggers] 설정으로 로깅을 맞춘다.
config = context.config
if config.config_file_name is not None:
    fileConfig(config.config_file_name)

target_metadata = Base.metadata


def run_migrations_offline() -> None:
    """DB에 연결하지 않고 적용할 SQL 문만 만들어 출력한다(`alembic upgrade --sql`)."""
    context.configure(
        url=config.get_main_option("sqlalchemy.url"),
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "named"},
        render_as_batch=True,
    )
    with context.begin_transaction():
        context.run_migrations()


def run_migrations_online() -> None:
    """실제 DB에 연결해 아직 적용하지 않은 마이그레이션을 차례로 적용한다."""
    # 마이그레이션은 한 번 돌고 끝나므로 연결을 풀에 남겨 두지 않는다(NullPool).
    connectable = engine_from_config(
        config.get_section(config.config_ini_section, {}),
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )

    @event.listens_for(connectable, "connect")
    def _enable_foreign_keys(dbapi_connection: object, _connection_record: object) -> None:
        """새 연결마다 SQLite 외래 키 검사를 켠다(SQLite 기본값은 꺼짐)."""
        cursor = dbapi_connection.cursor()  # type: ignore[attr-defined]
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.close()

    with connectable.connect() as connection:
        context.configure(
            connection=connection,
            target_metadata=target_metadata,
            render_as_batch=True,
        )
        with context.begin_transaction():
            context.run_migrations()


# alembic 명령이 이 파일을 불러 실행할 때, 모드에 맞는 쪽을 고른다.
if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()
