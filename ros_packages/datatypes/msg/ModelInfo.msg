# Static / registry description of a vision model in the model store.
string model_id
string task
string licence          # may be "unknown - see source" (registry policy)
int32 shaves
uint32 size_bytes
bool available          # present in the model store
bool active
