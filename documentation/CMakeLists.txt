#------------------------------------------------------------------
#
# Set up a target for generating the doxygen documentation
#
#
# FindDoxygen
find_package(Doxygen)

if(Doxygen_FOUND)
  set(DOXYGEN_GENERATE_HTML YES)
  set(DOXYGEN_GENERATE_MAN NO)

  doxygen_add_docs(
      apidoc
      ALL
      ${PROJECT_SOURCE_DIR}/OpenTissue
  )

endif(Doxygen_FOUND)


