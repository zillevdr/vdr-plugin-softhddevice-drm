///
///	@file softhddevice.h	@brief software HD device plugin header file.
///
///	Copyright (c) 2011, 2014 by Johns.  All Rights Reserved.
///	Copyright (c) 2018 - 2019 zille.  All Rights Reserved.
///
///	Contributor(s):
///
///	License: AGPLv3
///
///	This program is free software: you can redistribute it and/or modify
///	it under the terms of the GNU Affero General Public License as
///	published by the Free Software Foundation, either version 3 of the
///	License.
///
///	This program is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
///	GNU Affero General Public License for more details.
///
///	$Id$
//////////////////////////////////////////////////////////////////////////////

    /// vdr-plugin version number.
    /// Makefile extracts the version number for generating the file name
    /// for the distribution archive.
static const char *const VERSION = "0.0.1rc1"
#ifdef GIT_REV
    "-GIT" GIT_REV
#endif
    ;

    /// vdr-plugin description.
static const char *const DESCRIPTION =
trNOOP("A software and GPU emulated HD device");

    /// vdr-plugin text of main menu entry
static const char *MAINMENUENTRY = trNOOP("SoftHdDevice-drm");

    /// single instance of softhddevice plugin device.
static class cSoftHdDevice *MyDevice;

//////////////////////////////////////////////////////////////////////////////

static char ConfigMakePrimary;		///< config primary wanted
static char ConfigHideMainMenuEntry;	///< config hide main menu entry

char ConfigSWDeinterlacer;			///< config use sw deinterlacer

static int ConfigVideoAudioDelay;	///< config audio delay
static char ConfigAudioPassthrough;	///< config audio pass-through mask
static char AudioPassthroughState;	///< flag audio pass-through on/off
static char ConfigAudioDownmix;		///< config ffmpeg audio downmix
static char ConfigAudioSoftvol;		///< config use software volume
static char ConfigAudioNormalize;	///< config use normalize volume
static int ConfigAudioMaxNormalize;	///< config max normalize factor
static char ConfigAudioCompression;	///< config use volume compression
static int ConfigAudioMaxCompression;	///< config max volume compression
static int ConfigAudioStereoDescent;	///< config reduce stereo loudness
int ConfigAudioBufferTime;			///< config size ms of audio buffer
static int ConfigAudioAutoAES;		///< config automatic AES handling
static int ConfigAudioEq;			///< config equalizer filter 
static int SetupAudioEqBand[18];	///< config equalizer filter bands

static volatile int DoMakePrimary;	///< switch primary device to this

//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//	OSD
//////////////////////////////////////////////////////////////////////////////

/**
**	Soft device plugin OSD class.
*/
class cSoftOsd:public cOsd
{
  public:
    static volatile char Dirty;		///< flag force redraw everything
    int OsdLevel;			///< current osd level FIXME: remove

     cSoftOsd(int, int, uint);		///< osd constructor
     virtual ~ cSoftOsd(void);		///< osd destructor
    /// set the sub-areas to the given areas
    virtual eOsdError SetAreas(const tArea *, int);
    virtual void Flush(void);		///< commits all data to the hardware
    virtual void SetActive(bool);	///< sets OSD to be the active one
};

volatile char cSoftOsd::Dirty;		///< flag force redraw everything


//////////////////////////////////////////////////////////////////////////////
//	OSD provider
//////////////////////////////////////////////////////////////////////////////

/**
**	Soft device plugin OSD provider class.
*/
class cSoftOsdProvider:public cOsdProvider
{
  private:
    static cOsd *Osd;			///< single OSD
  public:
    virtual cOsd * CreateOsd(int, int, uint);
    virtual bool ProvidesTrueColor(void);
    cSoftOsdProvider(void);		///< OSD provider constructor
    //virtual ~cSoftOsdProvider();	///< OSD provider destructor
};

cOsd *cSoftOsdProvider::Osd;		///< single osd


//////////////////////////////////////////////////////////////////////////////
//	cMenuSetupPage
//////////////////////////////////////////////////////////////////////////////

/**
**	Soft device plugin menu setup page class.
*/
class cMenuSetupSoft:public cMenuSetupPage
{
  protected:
    ///
    /// local copies of global setup variables:
    /// @{
    int General;
    int MakePrimary;
    int HideMainMenuEntry;

    int Video;
    int SWDeinterlacer;

    int Audio;
    int AudioDelay;
    int AudioPassthroughDefault;
    int AudioPassthroughPCM;
    int AudioPassthroughAC3;
    int AudioPassthroughEAC3;
    int AudioDownmix;
    int AudioSoftvol;
    int AudioNormalize;
    int AudioMaxNormalize;
    int AudioCompression;
    int AudioMaxCompression;
    int AudioStereoDescent;
    int AudioBufferTime;
    int AudioAutoAES;
    int AudioFilter;
    int AudioEq;
    int AudioEqBand[18];

    /// @}
  private:
     inline cOsdItem * CollapsedItem(const char *, int &, const char * = NULL);
    void Create(void);			// create sub-menu
  protected:
     virtual void Store(void);
  public:
     cMenuSetupSoft(void);
    virtual eOSState ProcessKey(eKeys);	// handle input
};

//////////////////////////////////////////////////////////////////////////////
//	cOsdMenu
//////////////////////////////////////////////////////////////////////////////

/**
**	Hotkey parsing state machine.
*/
typedef enum
{
    HksInitial,				///< initial state
    HksBlue,				///< blue button pressed
    HksBlue1,				///< blue and 1 number pressed
    HksRed,				///< red button pressed
} HkState;

/**
**	Soft device plugin menu class.
*/
class cSoftHdMenu:public cOsdMenu
{
  private:
    HkState HotkeyState;		///< current hot-key state
    int HotkeyCode;			///< current hot-key code
    void Create(void);			///< create plugin main menu
  public:
    cSoftHdMenu(const char *, int = 0, int = 0, int = 0, int = 0, int = 0);
    virtual ~ cSoftHdMenu();
    virtual eOSState ProcessKey(eKeys);
};

//////////////////////////////////////////////////////////////////////////////
//	cDevice
//////////////////////////////////////////////////////////////////////////////

class cSoftHdDevice:public cDevice
{
  public:
    cSoftHdDevice(void);
    virtual ~ cSoftHdDevice(void);

    virtual bool HasDecoder(void) const;
    virtual bool CanReplay(void) const;
    virtual bool SetPlayMode(ePlayMode);
    virtual void TrickSpeed(int, bool);
    virtual void Clear(void);
    virtual void Play(void);
    virtual void Freeze(void);
    virtual void Mute(void);
    virtual void StillPicture(const uchar *, int);
    virtual bool Poll(cPoller &, int = 0);
    virtual bool Flush(int = 0);
    virtual int64_t GetSTC(void);
    virtual void SetVideoDisplayFormat(eVideoDisplayFormat);
    virtual void SetVideoFormat(bool);
    virtual void GetVideoSize(int &, int &, double &);
    virtual void GetOsdSize(int &, int &, double &);
    virtual int PlayVideo(const uchar *, int);
    virtual int PlayAudio(const uchar *, int, uchar);
    virtual void SetAudioChannelDevice(int);
    virtual int GetAudioChannelDevice(void);
    virtual void SetDigitalAudioDevice(bool);
    virtual void SetAudioTrackDevice(eTrackType);
    virtual void SetVolumeDevice(int);

// Image Grab facilities

    virtual uchar *GrabImage(int &, bool, int, int, int);

// SPU facilities
  private:
    cDvbSpuDecoder * spuDecoder;
  public:
    virtual cSpuDecoder * GetSpuDecoder(void);

  protected:
    virtual void MakePrimaryDevice(bool);
};

//////////////////////////////////////////////////////////////////////////////
//	cPlugin
//////////////////////////////////////////////////////////////////////////////

class cPluginSoftHdDevice:public cPlugin
{
  public:
    cPluginSoftHdDevice(void);
    virtual ~ cPluginSoftHdDevice(void);
    virtual const char *Version(void);
    virtual const char *Description(void);
    virtual const char *CommandLineHelp(void);
    virtual bool ProcessArgs(int, char *[]);
    virtual bool Initialize(void);
    virtual bool Start(void);
    virtual void Stop(void);
    virtual const char *MainMenuEntry(void);
    virtual cOsdObject *MainMenuAction(void);
    virtual cMenuSetupPage *SetupMenu(void);
    virtual bool SetupParse(const char *, const char *);
    virtual bool Service(const char *, void * = NULL);
    virtual const char **SVDRPHelpPages(void);
    virtual cString SVDRPCommand(const char *, const char *, int &);
};

