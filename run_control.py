from krpc_client.telemetry_reader import connect_to_ksp, get_data_from_vessel

conn        = connect_to_ksp()
vessel      = conn.space_center.active_vessel


# previous_time = time.time()

# while True:
#     current_time = time.time()
#     dt = current_time - previous_time 
#     previous_time = current_time

data        = get_data_from_vessel(conn, vessel)
print(data)