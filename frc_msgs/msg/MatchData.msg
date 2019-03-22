# Corresponds to frc::DriverStation::MatchType
uint8 MATCH_TYPE_NONE = 0
uint8 MATCH_TYPE_PRACTICE = 1
uint8 MATCH_TYPE_QUALIFICATION = 2
uint8 MATCH_TYPE_ELIMINATION = 3

# Corresponds to frc::DriverStation::Alliance
uint8 ALLIANCE_RED = 0
uint8 ALLIANCE_BLUE = 1
uint8 ALLIANCE_INVALID = 2

# Corresponds to return of frc::DriverStation::GetLocation()
uint8 LOCATION_INVALID = 0
uint8 LOCATION_LEFT = 1
uint8 LOCATION_CENTER = 2
uint8 LOCATION_RIGHT = 3

Header header
string game_specific_message
string event_name
uint8 match_type
int32 match_number
int32 replay_number
uint8 alliance
uint8 location
