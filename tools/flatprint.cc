#include <iostream>
#include <string>
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/idl.h"
#include "flatbuffers/util.h"

#include "sample_log_generated.h"

void parse_nested_container(const flatbuffers::Parser& parser, const void* root_flatbuffer, std::string* text) {
    auto sample_log = flatbuffers::GetRoot<fbs::SampleLog>(root_flatbuffer);
    auto samples = sample_log->samples();

    *text += "samples:\n";
    for (auto s: *samples) {
        flatbuffers::GenerateText(parser, s->data()->Data(), flatbuffers::GeneratorOptions(), text);
    }
}


int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <sample_schema> <container_binary>" << std::endl;
        std::cerr << "\nConvert a container flatbuffer sample binary to a text file with a " <<
            "sequence of the nested sample types." << std::endl;
        return EXIT_FAILURE;
    }

    flatbuffers::Parser parser;
    std::string schemafile;
    if (!flatbuffers::LoadFile(argv[1], true, &schemafile)) {
        std::cerr << "Unable to open schema: " << argv[1] << std::endl;
        return EXIT_FAILURE;
    }

    std::string buffer;
    if (!flatbuffers::LoadFile(argv[2], true, &buffer)) {
        std::cerr << "Unable to open binary: " << argv[2] << std::endl;
        return EXIT_FAILURE;
    }

    const char *include_directories[] = { flatbuffers::StripFileName(argv[1]).c_str(), "samples", nullptr };
    if (!parser.Parse(schemafile.c_str(), include_directories)) {
        std::cerr << "Unable to parse schema: " << argv[1] << std::endl;
        return EXIT_FAILURE;
    }

    if (!parser.SetRootType("Sample")) {
        std::cerr << "Unable to set parser root type to Sample" << std::endl;
        return EXIT_FAILURE;
    }

    std::string json_gen;
    parse_nested_container(parser, buffer.data(), &json_gen);
    std::cout << json_gen << std::endl;
    return EXIT_SUCCESS;
}
