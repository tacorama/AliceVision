# Headers
set(tracks_files_headers
  Track.hpp
  TracksHandler.hpp
  TracksBuilder.hpp
  tracksUtils.hpp
  trackIO.hpp
)

# Sources
set(tracks_files_sources
  TracksHandler.cpp
  TracksBuilder.cpp
  tracksUtils.cpp
  trackIO.cpp
)

alicevision_add_library(aliceVision_track
  SOURCES ${tracks_files_headers} ${tracks_files_sources}
  PUBLIC_LINKS
    aliceVision_feature
    aliceVision_matching
    aliceVision_stl
    aliceVision_sfmData
    aliceVision_dataio
    Boost::json
  PRIVATE_LINKS
    ${LEMON_LIBRARY}
)

# Unit tests
alicevision_add_test(track_test.cpp NAME "track" LINKS aliceVision_track)
