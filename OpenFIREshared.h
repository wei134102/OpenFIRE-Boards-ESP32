/*!
* @file  OpenFIREshared.h
* @brief Shared board assets for use between OpenFIRE microcontroller clients
*        and configuration apps for the OpenFIRE platform.
*
* @copyright That One Seong, 2025
*
*  OpenFIREshared is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef _OPENFIRESHARED_H_
#define _OPENFIRESHARED_H_

#include <string>
#include <map>
#include <unordered_map>
#include <vector>

//// BOARD IDENTIFIERS (for Desktop App identification and determining presets)

#ifdef ARDUINO_ADAFRUIT_ITSYBITSY_RP2040
#define OPENFIRE_BOARD "adafruitItsyRP2040"
#elifdef ARDUINO_ADAFRUIT_KB2040_RP2040
#define OPENFIRE_BOARD "adafruitKB2040"
#elifdef ARDUINO_NANO_RP2040_CONNECT
#define OPENFIRE_BOARD "arduinoNanoRP2040"
#elifdef ARDUINO_WAVESHARE_RP2040_ZERO
#define OPENFIRE_BOARD "waveshareZero"
#elifdef ARDUINO_YD_RP2040
#define OPENFIRE_BOARD "vccgndYD"
#elifdef ARDUINO_RASPBERRY_PI_PICO
#define OPENFIRE_BOARD "rpipico"
#elifdef ARDUINO_RASPBERRY_PI_PICO_W
#define OPENFIRE_BOARD "rpipicow"
#elifdef ARDUINO_ESP32_S3_WROOM1_DevKitC_1_N16R8
    #define OPENFIRE_BOARD "esp32-s3-devkitc-1"
#elifdef ARDUINO_WAVESHARE_ESP32_S3_PICO
    #define OPENFIRE_BOARD "waveshare-esp32-s3-pico"
#else
#define OPENFIRE_BOARD "generic"
#endif // board

class OF_Const
{
public:
    // Any new non-btn slots should ideally be added at the bottom, above the "count" line
    // Inputs Map indices
    enum {
        unavailable = -2,
        btnUnmapped = -1,
        btnTrigger = 0,
        btnGunA,
        btnGunB,
        btnGunC,
        btnStart,
        btnSelect,
        btnGunUp,
        btnGunDown,
        btnGunLeft,
        btnGunRight,
        btnPedal,
        btnPedal2,
        btnPump,
        btnHome,
        // ^ btn inputs
        rumblePin,
        solenoidPin,
        rumbleSwitch,
        solenoidSwitch,
        autofireSwitch,
        neoPixel,
        ledR,
        ledG,
        ledB,
        camSDA,
        camSCL,
        periphSDA,
        periphSCL,
        analogX,
        analogY,
        tempPin,
        wiiClockGen,
        // Add non-button inputs here
        boardInputsCount
    } boardInputs_e;

    // Note: names should be written in plain english,
    // as these name strings are shared with Apps
    const std::unordered_map<std::string, int> boardInputs_Strings = {
        {"Unmapped",            btnUnmapped     },
        {"Trigger",             btnTrigger      },
        {"Button A",            btnGunA         },
        {"Button B",            btnGunB         },
        {"Button C",            btnGunC         },
        {"Start",               btnStart        },
        {"Select",              btnSelect       },
        {"D-Pad Up",            btnGunUp        },
        {"D-Pad Down",          btnGunDown      },
        {"D-Pad Left",          btnGunLeft      },
        {"D-Pad Right",         btnGunRight     },
        {"Pedal",               btnPedal        },
        {"Alt Pedal",           btnPedal2       },
        {"Pump Action",         btnPump         },
        {"Home Button",         btnHome         },
        {"Rumble Signal",       rumblePin       },
        {"Solenoid Signal",     solenoidPin     },
        {"Rumble Switch",       rumbleSwitch    },
        {"Solenoid Switch",     solenoidSwitch  },
        {"Autofire Switch",     autofireSwitch  },
        {"External NeoPixel",   neoPixel        },
        {"RGB LED Red",         ledR            },
        {"RGB LED Green",       ledG            },
        {"RGB LED Blue",        ledB            },
        {"Camera SDA",          camSDA          },
        {"Camera SCL",          camSCL          },
        {"Peripherals SDA",     periphSDA       },
        {"Peripherals SCL",     periphSCL       },
        {"Analog Stick X",      analogX         },
        {"Analog Stick Y",      analogY         },
        {"Temperature Sensor",  tempPin         },
        {"Wii Cam Clock",       wiiClockGen     },
    };

    // For Apps to use for lists of pin functions
    char* boardInputs_sortedStr[boardInputsCount+1];

    // Constructor
    OF_Const() {
#ifdef OF_APP // generate strings list for the available board inputs
        for(auto &func : boardInputs_Strings)
            boardInputs_sortedStr[func.second+1] = (char*)func.first.c_str();
#endif // OF_APP
    }

    // Boolean/toggle settings indices
    enum {
        customPins = 0,
        rumble,
        solenoid,
        autofire,
        simplePause,
        holdToPause,
        commonAnode,
        lowButtonsMode,
        rumbleFF,
        invertStaticPixels,
        i2cOLED,
        i2cOLEDaltAddr,
        // Add here
        boolTypesCount
    } boolTypes_e;

    const std::unordered_map<std::string, int> boolTypes_Strings = {
        {"CustomPins",          customPins          },
        {"Rumble",              rumble              },
        {"Solenoid",            solenoid            },
        {"Autofire",            autofire            },
        {"SimplePause",         simplePause         },
        {"HoldToPause",         holdToPause         },
        {"LEDAnode",            commonAnode         },
        {"LowButtons",          lowButtonsMode      },
        {"RumbFFB",             rumbleFF            },
        {"InvertStaticPixels",  invertStaticPixels  },
        {"I2COLEDEnabled",      i2cOLED             },
        {"I2COLEDAltAddr",      i2cOLEDaltAddr      },
    };

    // Variable settings indices
    enum {
        rumbleStrength = 0,
        rumbleInterval,
        solenoidOnLength,
        solenoidOffLength,
        solenoidHoldLength,
        holdToPauseLength,
        customLEDcount,
        customLEDstatic,
        customLEDcolor1,
        customLEDcolor2,
        customLEDcolor3,
        tempWarning,
        tempShutdown,
        analogMode,
        // Add here
        settingsTypesCount
    } settingsTypes_e;

    const std::unordered_map<std::string, int> settingsTypes_Strings = {
        {"RumbPwr",             rumbleStrength      },
        {"RumbTime",            rumbleInterval      },
        {"SolOn",               solenoidOnLength    },
        {"SolOff",              solenoidOffLength   },
        {"SolHold",             solenoidHoldLength  },
        {"HoldToPauseLength",   holdToPauseLength   },
        {"CtmPixelsCount",      customLEDcount      },
        {"StaticPixels",        customLEDstatic     },
        {"StaticColor1",        customLEDcolor1     },
        {"StaticColor2",        customLEDcolor2     },
        {"StaticColor3",        customLEDcolor3     },
        {"TempWarning",         tempWarning         },
        {"TempDanger",          tempShutdown        },
        {"AnalogMode",          analogMode          },
    };

    enum {
        analogModeStick = 0,
        analogModeDpad,
        analogModeKeys
    } analogModeSettings_e;

    // Profile data type indices
    // this MUST match the order of ProfileData_s in (FW)OpenFIREprefs
    // as ProfData is accessed by struct offset.
    enum {
        profTopOffset = 0,
        profBottomOffset,
        profLeftOffset,
        profRightOffset,
        profTLled,
        profTRled,
        profAdjX,
        profAdjY,
        profIrSens,
        profRunMode,
        profIrLayout,
        profColor,
        profName,
        profDataTypes,
        profCurrent = 0xFD,
    } profSyncTypes_e;

    const std::unordered_map<std::string, int> profSettingTypes_Strings = {
        {"TopOffset",   profTopOffset       },
        {"BtmOffset",   profBottomOffset    },
        {"LftOffset",   profLeftOffset      },
        {"RhtOffset",   profRightOffset     },
        {"TLLed",       profTLled           },
        {"TRLed",       profTRled           },
        {"AdjX",        profAdjX            },
        {"AdjY",        profAdjY            },
        {"IrSens",      profIrSens          },
        {"IrRunMode",   profRunMode         },
        {"IrLayout",    profIrLayout        },
        {"Color",       profColor           },
        {"Name",        profName            },
        {"CurrentProf", profCurrent         },
    };

    // Layout types indices
    enum {
        layoutSquare = 0,
        layoutDiamond,
        // Add here
        layoutTypes
    } layoutTypes_e;

    /* ////
     * Shared serial control/signal codes for both boards and app.
     * For purposes of app-side debugability: ASCII 128+ should be for the board to send,
     * and invisible ASCII characters/control codes 0-32 should be for the app to send.
     *
     * ASCII 33-127 should be avoided whenever possible.
     */////
    enum {
        // Docking commands
        sDock1 = 1,
        sDock2,

        // Mode toggles from app
        sIRTest = 5,
        sCaliProfile,
        sCaliStart,
        sCaliSens,
        sCaliLayout,

        // Test signals from app
        sTestSolenoid = 15,
        sTestRumble,
        sTestLEDR,
        sTestLEDG,
        sTestLEDB,

        // 696969 inseriti provvesoriamente per abilitare disabilitare invio dati in modalità docked
        sDockedSaving = 69,
        sDockedSaving_on,
        sDockedSaving_off,
        // 696969 inseriti provvisoriamente

        // Error types from board (with sError)
        sErrCam = 0x80, // 128
        sErrPeriphGeneric,

        // Status updates from board
        sBtnPressed = 0x90, // 144
        sBtnReleased,
        sAnalogPosUpd,
        sTemperatureUpd,
        sCaliStageUpd,
        sCaliInfoUpd,
        sTestCoords,
        sCurrentProf,

        // Push settings to board
        sCommitStart = 0xAA, // 170
        sCommitToggles,
        sCommitPins,
        sCommitSettings,
        sCommitProfile,
        sCommitID,

        // Grab settings from board
        sGetPins = 0xC8, // 200
        sGetToggles,
        sGetSettings,
        sGetProfile,

        sError = 0xFA, // 250
        sSave = 0xFC, // 252
        sClearFlash = 0xFD, // 253
        // Terminates out of any current mode, or undocks
        serialTerminator = 0xFE // 254
    } serialCmdTypes_e;

    enum {
        usbPID = 0,
        usbName,
    } usbIdSyncTypes_e;

    /// @brief      Map of default pin mappings for each supported board
    /// @details    Key = board, int array maps to RP2040 GPIO where each value is a FW function (or unmapped).
    const std::unordered_map<std::string, std::vector<int>> boardsPresetsMap = {
        //=====================================================================================================
        // Notes: rpi boards do not expose pins 23-25; pin 29/A3 is used for builtin chipset temp monitor
        {"rpipico",             {   btnGunA,       btnGunB,        btnGunC,        btnStart,       btnSelect,
                                    btnHome,       btnGunUp,       btnGunDown,     btnGunLeft,     btnGunRight,
                                    ledR,          ledG,           ledB,           btnPump,        btnPedal,
                                    btnTrigger,    solenoidPin,    rumblePin,      periphSDA,      periphSCL,
                                    camSDA,        camSCL,         btnUnmapped,    unavailable,    unavailable,
                                    unavailable,   btnUnmapped,    btnUnmapped,    tempPin,        unavailable}},
        //=====================================================================================================
        // Notes: rpi boards do not expose pins 23-25; pin 29/A3 is used for builtin chipset temp monitor
        {"rpipicow",            {   btnGunA,       btnGunB,        btnGunC,        btnStart,       btnSelect,
                                    btnHome,       btnGunUp,       btnGunDown,     btnGunLeft,     btnGunRight,
                                    periphSDA,     periphSCL,      btnUnmapped,    btnPump,        btnPedal,
                                    btnTrigger,    solenoidPin,    rumblePin,      btnUnmapped,    btnUnmapped,
                                    camSDA,        camSCL,         btnUnmapped,    unavailable,    unavailable,
                                    unavailable,   analogY,        analogX,        tempPin,        unavailable}},
        //=====================================================================================================
        // Notes: pins 13-17 & 21-23 are unexposed
        {"adafruitItsyRP2040",  {   btnUnmapped,   btnUnmapped,    camSDA,         camSCL,         btnPedal,
                                    btnUnmapped,   btnTrigger,     btnGunDown,     btnGunLeft,     btnGunUp,
                                    btnGunRight,   btnGunC,        btnUnmapped,    unavailable,    unavailable,
                                    unavailable,   unavailable,    unavailable,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   unavailable,    unavailable,    unavailable,    rumblePin,
                                    solenoidPin,   btnGunB,        btnGunA,        btnStart,       btnSelect}},
        //=====================================================================================================
        // Notes: pins 11-17 & 21-25 are unexposed
        {"adafruitKB2040",      {   btnUnmapped,   btnUnmapped,    camSDA,         camSCL,         btnGunB,
                                    rumblePin,     btnGunC,        solenoidPin,    btnSelect,      btnStart,
                                    btnGunRight,   unavailable,    unavailable,    unavailable,    unavailable,
                                    unavailable,   unavailable,    unavailable,    btnGunUp,       btnGunLeft,
                                    btnGunDown,    unavailable,    unavailable,    unavailable,    unavailable,
                                    unavailable,   tempPin,        btnHome,        btnTrigger,     btnGunA}},
        //=====================================================================================================
        // Notes: pins 2-3, 8-11, 14, & 22-24 are unexposed;
        //        some other pins are analog, but controlled by NINA and thus are unavailable in OpenFIRE for the moment
        {"arduinoNanoRP2040",   {   btnTrigger,    btnPedal,       unavailable,    unavailable,    btnGunA,
                                    btnGunC,       btnUnmapped,    btnGunB,        unavailable,    unavailable,
                                    unavailable,   unavailable,    camSDA,         camSCL,         unavailable,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    unavailable,    unavailable,    unavailable,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    tempPin,        btnUnmapped}},
        //=====================================================================================================
        // Note: pin 16 is reserved for the board's builtin NeoPixel (not currently used?);
        //       pins 17-25 are underside pads which are not exposed in the app for layout reasons;
        {"waveshareZero",       {   btnTrigger,    btnGunA,        btnGunB,        btnGunC,        btnStart,
                                    btnSelect,     btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    camSDA,
                                    camSCL,        unavailable,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    tempPin}},
        //=====================================================================================================
        // Notes: ESP32          /*xx*/ indicates the number of the GPIO es. /*02*/ for GPIO2
        {"esp32-s3-devkitc-1",         {/*00*/unavailable,   /*01*/btnTrigger,     /*02*/btnGunRight,    /*03*/btnUnmapped,    /*04*/analogX,
                                        /*05*/analogY,       /*06*/tempPin,        /*07*/btnUnmapped,    /*08*/camSDA,         /*09*/camSCL,
                                        /*10*/btnUnmapped,   /*11*/btnUnmapped,    /*12*/btnUnmapped,    /*13*/btnUnmapped,    /*14*/btnUnmapped,
                                        /*15*/periphSCL,     /*16*/rumblePin,      /*17*/solenoidPin,    /*18*/periphSDA,      /*19*/unavailable,
                                        /*20*/unavailable,   /*21*/btnGunC,        /*22*/unavailable,    /*23*/unavailable,    /*24*/unavailable,
                                        /*25*/unavailable,   /*26*/unavailable,    /*27*/unavailable,    /*28*/unavailable,    /*29*/unavailable,
                                        /*30*/unavailable,   /*31*/unavailable,    /*32*/unavailable,    /*33*/unavailable,    /*34*/unavailable,
                                        /*35*/btnHome,       /*36*/btnGunA,        /*37*/btnGunB,        /*38*/btnSelect,      /*39*/btnStart,
                                        /*40*/btnGunUp,      /*41*/btnGunDown,     /*42*/btnGunLeft,     /*43*/unavailable,    /*44*/unavailable,
                                        /*45*/btnPump,       /*46*/unavailable,    /*47*/btnPedal,       /*48*/neoPixel,       /*49*/unavailable}},
        //=====================================================================================================
        // Notes: ESP32          /*xx*/ indicates the number of the GPIO es. /*02*/ for GPIO2
        {"waveshare-esp32-s3-pico",    {/*00*/unavailable,   /*01*/btnUnmapped,    /*02*/btnUnmapped,    /*03*/unavailable,    /*04*/camSDA,
                                        /*05*/camSCL     ,   /*06*/btnUnmapped,    /*07*/analogY,        /*08*/analogX,        /*09*/tempPin,
                                        /*10*/btnUnmapped,   /*11*/btnGunA,        /*12*/btnGunB,        /*13*/btnGunC,        /*14*/btnStart,
                                        /*15*/btnSelect,     /*16*/btnHome,        /*17*/btnGunUp,       /*18*/btnGunDown,     /*19*/unavailable,
                                        /*20*/unavailable,   /*21*/neoPixel,       /*22*/unavailable,    /*23*/unavailable,    /*24*/unavailable,
                                        /*25*/unavailable,   /*26*/unavailable,    /*27*/unavailable,    /*28*/unavailable,    /*29*/unavailable,
                                        /*30*/unavailable,   /*31*/unavailable,    /*32*/unavailable,    /*33*/btnGunLeft,     /*34*/btnGunRight,
                                        /*35*/periphSDA,     /*36*/periphSCL,      /*37*/btnUnmapped,    /*38*/btnPump,        /*39*/btnPedal,
                                        /*40*/btnTrigger,    /*41*/rumblePin,      /*42*/solenoidPin,    /*43*/unavailable,    /*44*/unavailable,
                                        /*45*/unavailable,   /*46*/unavailable,    /*47*/unavailable,    /*48*/unavailable,    /*49*/unavailable}},
        //=====================================================================================================
    };

// Only needed for the Desktop App, don't build for microcontroller firmware!
#ifdef OF_APP

    const std::map<std::string, const char *> boardNames = {
        {"rpipico",             "Raspberry Pi Pico (RP2040)"},
        {"rpipicow",            "Raspberry Pi Pico W (RP2040)"},
        {"adafruitItsyRP2040",  "Adafruit ItsyBitsy RP2040"},
        {"adafruitKB2040",      "Adafruit Keeboar KB2040"},
        {"arduinoNanoRP2040",   "Arduino Nano Connect RP2040"},
        {"waveshareZero",       "Waveshare Zero RP2040"},
        {"esp32-s3-devkitc-1",      "Esp32-S3 Devkitc-1 N16R8"},
        {"waveshare-esp32-s3-pico", "Waveshare Esp32-S3-pico"},
        // Add more here!
        {"generic",             "Unknown Board"}
    };

    enum {
        posNothing = 0,
        posLeft = 512,
        posRight = 1024,
        posMiddle = 2048
    } boardBoxPositions_e;

    /// @brief      Map of graphical placement for each pin in the application
    /// @details    Key = board, int vector maps to microcontroller GPIO.
    ///             Each pin should be a combination of grid layout slot it should be in,
    ///             added by the grid it should belong to.
    ///             Unexposed pins should use only posNothing (0).
    ///             (Yep, bitpacking! Three least significant bits of the second byte determine left/right/under position)
    ///             (If anyone is aware of a better way of doing this, please let me know/send a PR!)
    const std::unordered_map<std::string, std::vector<unsigned int>> boardsBoxPositions = {
        //=====================================================================================================
        // Raspberry Pi Pico: 15 pins left, rest of the pins right. Mostly linear order save for the reserved pins.
        // Notes: rpi boards do not expose pins 23-25; pin 29/A3 is used for builtin chipset temp monitor
        {"rpipico",             {   1+posLeft,     2+posLeft,      4+posLeft,      5+posLeft,      6+posLeft,
                                    7+posLeft,     9+posLeft,      10+posLeft,     11+posLeft,     12+posLeft,
                                    14+posLeft,    15+posLeft,     16+posLeft,     17+posLeft,     19+posLeft,
                                    20+posLeft,    20+posRight,    19+posRight,    17+posRight,    16+posRight,
                                    15+posRight,   14+posRight,    12+posRight,    posNothing,     posNothing,
                                    posNothing,    10+posRight,    9+posRight,     7+posRight,     posNothing}},
        //=====================================================================================================
        // Raspberry Pi Pico W: same as non-W Pico.
        // Notes: rpi boards do not expose pins 23-25; pin 29/A3 is used for builtin chipset temp monitor
        {"rpipicow",            {   1+posLeft,     2+posLeft,      4+posLeft,      5+posLeft,      6+posLeft,
                                    7+posLeft,     9+posLeft,      10+posLeft,     11+posLeft,     12+posLeft,
                                    14+posLeft,    15+posLeft,     16+posLeft,     17+posLeft,     19+posLeft,
                                    20+posLeft,    20+posRight,    19+posRight,    17+posRight,    16+posRight,
                                    15+posRight,   14+posRight,    12+posRight,    posNothing,     posNothing,
                                    posNothing,    10+posRight,    9+posRight,     7+posRight,     posNothing}},
        //=====================================================================================================
        // Adafruit ItsyBitsy RP2040: A very cluttered and kind of unfriendly layout tbh :(
        // Notes: pins 13-17 & 21-23 are unexposed
        {"adafruitItsyRP2040",  {   13+posRight,   14+posRight,    12+posRight,    11+posRight,    2+posMiddle,
                                    1+posMiddle,   9+posRight,     8+posRight,     7+posRight,     6+posRight,
                                    5+posRight,    4+posRight,     14+posLeft,     posNothing,     posNothing,
                                    posNothing,    posNothing,     posNothing,     11+posLeft,     12+posLeft,
                                    13+posLeft,    posNothing,     posNothing,     posNothing,     9+posLeft,
                                    10+posLeft,    5+posLeft,      6+posLeft,      7+posLeft,      8+posLeft}},
        //=====================================================================================================
        // Adafruit KB2040: Like the Itsy with more padding.
        // Notes: pins 11-17 & 21-25 are unexposed
        {"adafruitKB2040",      {   3+posLeft,     4+posLeft,      7+posLeft,      8+posLeft,      9+posLeft,
                                    10+posLeft,    11+posLeft,     12+posLeft,     13+posLeft,     14+posLeft,
                                    14+posRight,   posNothing,     posNothing,     posNothing,     posNothing,
                                    posNothing,    posNothing,     posNothing,     11+posRight,    13+posRight,
                                    12+posRight,   posNothing,     posNothing,     posNothing,     posNothing,
                                    posNothing,    10+posRight,    9+posRight,     8+posRight,     7+posRight}},
        //=====================================================================================================
        // Arduino Nano RP2040: gweh
        // Notes: pins 2-3, 8-11, 14, & 22-24 are unexposed;
        //        some other pins are analog, but controlled by NINA and thus are unavailable in OpenFIRE for the moment
        {"arduinoNanoRP2040",   {   18+posRight,   17+posRight,    posNothing,     posNothing,     4+posRight,
                                    6+posRight,    4+posLeft,      5+posRight,     posNothing,     posNothing,
                                    posNothing,    posNothing,     11+posLeft,     12+posLeft,     posNothing,
                                    13+posRight,   12+posRight,    11+posRight,    10+posRight,    9+posRight,
                                    8+posRight,    7+posRight,     posNothing,     posNothing,     posNothing,
                                    14+posRight,   7+posLeft,      8+posLeft,      9+posLeft,      10+posLeft}},
        //=====================================================================================================
        // Waveshare Zero RP2040: Clockwise layout
        // Note: pin 16 is reserved for the board's builtin NeoPixel (not currently used?);
        //       pins 17-25 are underside pads which are not exposed in the app for layout reasons;
        {"waveshareZero",       {   2+posRight,    3+posRight,     4+posRight,     5+posRight,     6+posRight,
                                    7+posRight,    8+posRight,     9+posRight,     10+posRight,    11+posRight,
                                    3+posMiddle,   2+posMiddle,    11+posLeft,     10+posLeft,     9+posLeft,
                                    8+posLeft,     posNothing,     posNothing,     posNothing,     posNothing,
                                    posNothing,    posNothing,     posNothing,     posNothing,     posNothing,
                                    posNothing,    7+posLeft,      6+posLeft,      5+posLeft,      4+posLeft}},
        //=====================================================================================================
        // Insert new layouts below this one!
        // Feel free to use any of the above as a template.
        // ***
        
        
        //=====================================================================================================
        // Esp32 S3 Devkitc 1
        // Notes: Esp32          /*xx*/ indicates the number of the GPIO es. /*02*/ for GPIO2
        {"esp32-s3-devkitc-1",         {/*00*/posNothing,    /*01*/19+posLeft,     /*02*/18+posLeft,     /*03*/posNothing,     /*04*/19+posRight,
                                        /*05*/18+posRight,   /*06*/17+posRight,    /*07*/posNothing,     /*08*/11+posRight,    /*09*/8+posRight,
                                        /*10*/posNothing,    /*11*/posNothing,     /*12*/posNothing,     /*13*/posNothing,     /*14*/posNothing,
                                        /*15*/15+posRight,   /*16*/14+posRight,    /*17*/13+posRight,    /*18*/12+posRight,    /*19*/posNothing,
                                        /*20*/posNothing,    /*21*/5+posLeft,      /*22*/posNothing,     /*23*/posNothing,     /*24*/posNothing,
                                        /*25*/posNothing,    /*26*/posNothing,     /*27*/posNothing,     /*28*/posNothing,     /*29*/posNothing,
                                        /*30*/posNothing,    /*31*/posNothing,     /*32*/posNothing,     /*33*/posNothing,     /*34*/posNothing,
                                        /*35*/10+posLeft,    /*36*/11+posLeft,     /*37*/12+posLeft,     /*38*/13+posLeft,     /*39*/14+posLeft,
                                        /*40*/15+posLeft,    /*41*/16+posLeft,     /*42*/17+posLeft,     /*43*/posNothing,     /*44*/posNothing,
                                        /*45*/8+posLeft,     /*46*/posNothing,     /*47*/6+posLeft,      /*48*/7+posLeft,      /*49*/posNothing}},
        //=====================================================================================================
        //=====================================================================================================
        // Notes: ESP32          /*xx*/ indicates the number of the GPIO es. /*02*/ for GPIO2
        {"waveshare-esp32-s3-pico",    {/*00*/posNothing,    /*01*/17+posRight,    /*02*/16+posRight,    /*03*/posNothing,     /*04*/15+posRight,
                                        /*05*/14+posRight,   /*06*/12+posRight,    /*07*/10+posRight,    /*08*/9+posRight,     /*09*/7+posRight,
                                        /*10*/6+posRight,    /*11*/1+posLeft,      /*12*/2+posLeft,      /*13*/4+posLeft,      /*14*/5+posLeft,
                                        /*15*/6+posLeft,     /*16*/7+posLeft,      /*17*/9+posLeft,      /*18*/10+posLeft,     /*19*/posNothing,
                                        /*20*/posNothing,    /*21*/posNothing,     /*22*/posNothing,     /*23*/posNothing,     /*24*/posNothing,
                                        /*25*/posNothing,    /*26*/posNothing,     /*27*/posNothing,     /*28*/posNothing,     /*29*/posNothing,
                                        /*30*/posNothing,    /*31*/posNothing,     /*32*/posNothing,     /*33*/11+posLeft,     /*34*/12+posLeft,
                                        /*35*/14+posLeft,    /*36*/15+posLeft,     /*37*/16+posLeft,     /*38*/17+posLeft,     /*39*/19+posLeft,
                                        /*40*/20+posLeft,    /*41*/19+posRight,    /*42*/20+posRight,    /*43*/posNothing,     /*44*/posNothing,
                                        /*45*/posNothing,    /*46*/posNothing,     /*47*/posNothing,     /*48*/posNothing,     /*49*/posNothing}},
//=====================================================================================================
        //=====================================================================================================
        // Generic layout
        // Just reveal all pins; user assumes full responsibility if something goes wrong here
        {"generic",             {   1+posLeft,     2+posLeft,      3+posLeft,      4+posLeft,      5+posLeft,
                                    6+posLeft,     7+posLeft,      8+posLeft,      9+posLeft,      10+posLeft,
                                    11+posLeft,    12+posLeft,     13+posLeft,     14+posLeft,     15+posLeft,
                                    16+posLeft,    16+posRight,    15+posRight,    14+posRight,    13+posRight,
                                    12+posRight,   11+posRight,    10+posRight,    9+posRight,     8+posRight,
                                    7+posRight,    6+posRight,     5+posRight,     4+posRight,     3+posRight}}
    };

    typedef struct {
        const char * name;
        std::vector<int> pin;
    } boardAltPresetsMap_t;

    /// @brief      MultiMap of alternative pin mappings for supported boards to show in the application.
    /// @details    Key = board (one board can be multiple), string literal label, int array maps to RP2040 GPIO where each value is a FW function (or unmapped).
    const std::unordered_multimap<std::string, boardAltPresetsMap_t> boardsAltPresets = {
        //=====================================================================================================
        // Raspberry Pi Pico Presets (currently a test)
        // Notes: rpi boards do not expose pins 23-25; pin 29/A3 is used for builtin chipset temp monitor
        {"rpipico",             {"Test",
                                {   btnPump,       btnPedal,       btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    unavailable,    unavailable,
                                    unavailable,   btnUnmapped,    btnUnmapped,    btnUnmapped,    unavailable}}},

        {"rpipico",             {"Test 2",
                                {   btnGunA,       btnTrigger,     btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnUnmapped,    unavailable,    unavailable,
                                    unavailable,   btnUnmapped,    btnUnmapped,    btnUnmapped,    unavailable}}},

        //=====================================================================================================
        // Adafruit ItsyBitsy RP2040 Presets
        // Notes: pins 13-17 & 21-23 are unexposed
        {"adafruitItsyRP2040",  {"SAMCO 2.0",
                                {   btnUnmapped,   btnUnmapped,    camSDA,         camSCL,         btnPedal,
                                    btnUnmapped,   btnTrigger,     btnGunDown,     btnGunLeft,     btnGunUp,
                                    btnGunRight,   btnHome,        btnUnmapped,    unavailable,    unavailable,
                                    unavailable,   unavailable,    unavailable,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   unavailable,    unavailable,    unavailable,    rumblePin,
                                    solenoidPin,   btnGunB,        btnGunA,        btnStart,       btnSelect}}},

        {"adafruitItsyRP2040",  {"SAMCO 1.1",
                                {   btnUnmapped,   btnUnmapped,    camSDA,         camSCL,         btnUnmapped,
                                    btnUnmapped,   btnGunA,        btnGunB,        rumblePin,      btnHome,
                                    btnTrigger,    btnUnmapped,    btnUnmapped,    unavailable,    unavailable,
                                    unavailable,   unavailable,    unavailable,    btnUnmapped,    btnUnmapped,
                                    btnUnmapped,   unavailable,    unavailable,    unavailable,    btnUnmapped,
                                    btnUnmapped,   btnUnmapped,    btnPedal,       btnUnmapped,    btnUnmapped}}},
    };

#endif
};


#endif // _OPENFIRESHARED_H_
