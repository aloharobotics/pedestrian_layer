FILE(REMOVE_RECURSE
  "CMakeFiles/foo.dir/src/foo.cpp.o"
  "../lib/libfoo.pdb"
  "../lib/libfoo.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/foo.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
