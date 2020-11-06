## Database Schema, Version 10

Warehouse stores its data in multiple databases and collections.
Each database can have one or more collection,
each collection can hold exactly one type of ROS Messages.
Each collection is stored in a separate SQLite table.
As SQLite only supports one SQL database per file,
the Warehouse database name and the collection name are mangled
together to get the name of the SQLite table.
The metadata of the messages (at least a unique id and the creation time)
are stored along the data in the SQLite table of every collection.
The table `WarehouseIndex` contains details of each collection and its mangled table name.
The version of the database scheme (currently 10) is stored in the [sqlite pragma](https://www.sqlite.org/pragma.html#pragma_user_version) `user_version`.

### Structure of a collection table

The name of the collection table is computed as follows:

- Replace every `@` symbol with `@@` of the warehouse database and collection name
- Join both names with a single `@` symbol
- Prepend the prefix `T_`

So, the name of the table for the collection `MyColl` in the database `MyDB@home` will be `T_MyDB@@home@MyColl`.

The metadata is stored in columns, their names are prefixed with `M_`.
If you insert a Message with the metadata `x=3` and `y="foo"`, the table will look like

| Data BLOB NOT NULL | M_id INTEGER | M_creation_time INTEGER | M_x INTEGER | M_y BLOB |
|--------------------|--------------|-------------------------|-------------|----------|
| &lt;binary data&gt;| 1            | 123456                  | 3           | "foo"    |

`M_ID` is `PRIMARY KEY AUTOINCREMENT`.
If a metadata value is missing, it will be `NULL` when the data is inserted into the table.
But when the data is read from the database,
`NULL` will be converted into the default constructed values (e.g. empty string, 0, 0.0) of the column.
The index of the `Data` column must always be 0.

### Structure of the Index

Beside the mapping between the SQLite tables and the Warehouse databases/collection,
the datatype from the Message of each collection (as taken from `ros::message_traits::<Msg>::DataType`)
and the MD5 sum of the message definition file (as taken from `ros::message_traits::<Msg>::MD5Sum`, see the ROS docs for details)
is stored.

| MangledTableName       | MessageMD5 BLOB    | WarehouseCollectionName | WarehouseDatabaseName | MessageDataType |
|------------------------|--------------------|-------------------------|-----------------------|-----------------|
| 'T_MyDB@@home@MyColl'  | &lt;MD5 binary&gt; | 'MyColl'                | 'MyDB@home'           | 'mymsgtype'     |

Each column is `NOT NULL`, and (except of the MD5 value) a `TEXT`. The `MangledTableName` is the primary key.
