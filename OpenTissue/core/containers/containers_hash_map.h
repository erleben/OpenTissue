#ifndef OPENTISSUE_CORE_CONTAINERS_CONTAINERS_HASH_MAP_H
#define OPENTISSUE_CORE_CONTAINERS_CONTAINERS_HASH_MAP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

//--- henrikd:
//--- Below code has been cut from http://gcc.gnu.org/onlinedocs/libstdc++/faq/index.html#5_4
//--- Hash on std::string hack has been cut from:
//---   http://www.gamedev.net/community/forums/topic.asp?topic_id=330897&forum_id=21&gforum_id=0
#ifdef __GNUC__
# if __GNUC__ < 3
#   include <hash_map.h>
    namespace stdext { using ::hash_map; }; // inherit globals
# else
#   include <ext/hash_map>
#   if (__GNUC__ == 3 && __GNUC_MINOR__ == 0)
      namespace stdext = std;               // GCC 3.0
#   else
#     include <cstddef>  // size_t
#     include <locale>   // use_facet, collate, locale
#     include <string>
      namespace __gnu_cxx
      {
        template< typename CharT, typename Traits, typename Alloc >
        struct hash< const std::basic_string<CharT, Traits, Alloc> >
        {
          std::size_t operator()(const std::basic_string<CharT, Traits, Alloc>& s) const
          {
            const std::collate<CharT>& c = std::use_facet<std::collate<CharT> >(std::locale());
            return c.hash(s.c_str(), s.c_str() + s.length());
          }
        };

        template< typename CharT, typename Traits, typename Alloc >
        struct hash< std::basic_string<CharT, Traits, Alloc> >
          : hash< const std::basic_string<CharT, Traits, Alloc> > {};
      }
//--- Hash on std::string hack has been cut from http://gcc.gnu.org/bugzilla/show_bug.cgi?id=13342
//       namespace __gnu_cxx {
//         template<> struct hash<std::string>
//         {
//           std::size_t operator()(const std::string& s) const
//           {
//             const std::collate<char>& c = std::use_facet<std::collate<char> >(std::locale::classic());
//             return c.hash(s.c_str(), s.c_str() + s.size());
//           }
//         };
//       }
       namespace stdext = __gnu_cxx;       // GCC 3.1 and later
#   endif
# endif
#else      // ...  there are other compilers, right?
#   include <hash_map>
//   namespace stdext = std;  // This is what the namespace is called in MSVC. Might need to extend this for other compilers.
#endif

//OPENTISSUE_CORE_CONTAINERS_CONTAINERS_HASH_MAP_H
#endif
