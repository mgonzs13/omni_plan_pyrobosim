# Order completion report published when a table is set FREE again.

# Unique identifier for this order (incremented per customer group)
int32 order_id

# Name of the table that was served
string table

# Number of customers in the group
int32 group_size

# Names of the item categories that were ordered (e.g. ["dish_order", "drink_order"])
string[] item_categories

# Total time from customer arrival (start of handle_customer_group) to
# table becoming FREE again, in seconds
float64 total_duration_sec

# Timestamp when the order started (customer arrival)
builtin_interfaces/Time start_time

# Timestamp when the order completed (table set FREE)
builtin_interfaces/Time end_time

# Names of all robots that were involved in serving this order
string[] robots_involved
