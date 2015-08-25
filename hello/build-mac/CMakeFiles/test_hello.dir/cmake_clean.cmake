FILE(REMOVE_RECURSE
  "CMakeFiles/test_hello.dir/test.cpp.o"
  "sdk/bin/test_hello.pdb"
  "sdk/bin/test_hello"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/test_hello.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
