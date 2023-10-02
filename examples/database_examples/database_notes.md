# Database Notes

## Overview

Basic Databases:
- relational (sql)
- document (mongodb), media data
- graph (neo4j), how customers are related to each other
- full text search: elastic
- Cache: redis

## Redis

1. Features
    - Schemaless
    - has RedisSearch, RedisGraph, etc. to support other datatypes
2. Persistence:
    - multiple copies
        - have primary and replicas; can have shards
    - store snapshots + Append only file to record write actions
        - to rebuild state
