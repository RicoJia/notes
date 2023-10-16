# Database Notes

## Overview

Basic Databases:
- relational (sql) (Structural)
  - SQL: more structure (table), requires a schema, scale vertically, relationships
        - relational: you have clear relationships, SQL is for you: table 1: user_id | user_address; table 2: order_id | user_id | product_id ; table3: product_id | product_description. Can set columns to be related to one another
        - But a table grows vertically. Searching there might hit a bottleneck; NoSQL can grow horizontally.
  - Mongodb: horizontal (more distributed.)
- document, no sql (mongodb), media data
- graph (neo4j), how customers are related to each other
- full text search: elastic
- Cache: redis

## Postgres

1. `docker compose up` to start the container
2. `docker exec -it elevator_service_db psql -U postgres`
    - psql is the postgreSQL client; -U is the user.
3. Basic Opertions

```
# Not sure if this is necessary
SELECT current_database()
# List of databases
\l
# connect to the database
\c DATABASE_NAME
# check tables
\dt
SELECT * FROM TABLE_NAME;
```

4. Syntax
    - end your command with `;`. Otherwise, each line will be moved to a buffer.
    - reset the buffer is `\r`


## Redis

1. Features
    - Schemaless
    - has RedisSearch, RedisGraph, etc. to support other datatypes
2. Persistence:
    - multiple copies
        - have primary and replicas; can have shards
    - store snapshots + Append only file to record write actions
        - to rebuild state
