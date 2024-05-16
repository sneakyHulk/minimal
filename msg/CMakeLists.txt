cmake_minimum_required(VERSION 3.22)

flatbuffers_generate_headers(
        TARGET CompactObject
        SCHEMAS CompactObject.fbs
        FLAGS --gen-object-api
)

flatbuffers_generate_headers(
        TARGET Detection2D
        SCHEMAS Detection2D.fbs
        FLAGS --gen-object-api
)

flatbuffers_generate_headers(
        TARGET Image
        SCHEMAS Image.fbs
        FLAGS --gen-object-api
)