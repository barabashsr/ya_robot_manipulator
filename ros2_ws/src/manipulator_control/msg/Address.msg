# Warehouse storage address specification
string side          # Cabinet side: "left" or "right"
uint8 cabinet_num    # Cabinet number (1-4)
uint8 row            # Row within cabinet (1-N, depends on cabinet configuration)
uint8 column         # Column within cabinet (1-N, depends on cabinet configuration)
