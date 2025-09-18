// takes raw byte array, matches the interface and parses the data
void store_data(std::vector<uint8_t> payload) {

    if (!payload.empty()) {

        // find id
        uint8_t id = payload[0];
    
        // parse the struct based on the id
        if (id == PID_CMD) {
            
            // function to parse the struct
            cmd_pidvals = parse_struct<PIDTest>(payload); // count.a, count.b, count.c, count.d, based on your interface
            new_data = true;
        }
    }
}
