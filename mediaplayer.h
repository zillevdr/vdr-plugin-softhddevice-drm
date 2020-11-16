///
///	@file mediaplayer.cpp	@brief A software HD device plugin for VDR.
///
///	Copyright (c) 2020 by zille.  All Rights Reserved.
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

	struct PLEntry {
		string Path;
		string File;
		string Folder;
		string SubFolder;
		struct PLEntry *NextEntry;
	};

//////////////////////////////////////////////////////////////////////////////
//	cPlayer
//////////////////////////////////////////////////////////////////////////////

/**
**	player for mediaplayer mode.
*/
class cSoftHdPlayer : public cPlayer, cThread
{
private:
	void Player(const char *);
	void ReadPL(const char *);
	char *Source;
	int Entries;
protected:
	virtual void Activate(bool On);
	virtual void Action(void);
public:
	cSoftHdPlayer(const char *);
	virtual ~ cSoftHdPlayer();
	struct PLEntry *FirstEntry;
	struct PLEntry *CurrentEntry;
	void SetEntry(int);
	const char * GetTitle(void);
	int Jump;
	int Pause;
	int StopPlay;
	int Random;
	int NoModify;
	int CurrentTime;
	int Duration;
};

//////////////////////////////////////////////////////////////////////////////
//	cControl
//////////////////////////////////////////////////////////////////////////////

/**
**	control class for mediaplayer mode.
*/

class cSoftHdControl : public cControl
{
private:
	void ShowProgress();
	static cSoftHdControl *pControl;
	static cSoftHdPlayer *pPlayer;
	cSkinDisplayReplay *pOsd;
public:
	cSoftHdControl(const char *);		///< control constructor
	virtual ~cSoftHdControl();		///< control destructor
	virtual void Hide(void);		///< hide control
	virtual cOsdObject *GetInfo(void) { return NULL; }
	virtual eOSState ProcessKey(eKeys);	///< process input events
	static cSoftHdControl *Control() { return pControl; }
	static cSoftHdPlayer *Player() { return pPlayer; }
	int Close;
};

//////////////////////////////////////////////////////////////////////////////
//	cOsdMenu
//////////////////////////////////////////////////////////////////////////////

/**
**	Soft device plugin menu class.
*/
class cSoftHdMenu : public cOsdMenu
{
private:
	void MainMenu(void);			///< create plugin main menu
	void SelectPL(void);
	void FindFile(string, FILE *);
	void MakePlayList(const char *, const char *);
	int TestMedia(const char *);
	void PlayMedia(const char *);
	string Path;
	string LastItem;
	string Playlist;
public:
	cSoftHdMenu(const char *, int = 0, int = 0, int = 0, int = 0, int = 0);
	virtual ~ cSoftHdMenu();
	void PlayListMenu(void);
	virtual eOSState ProcessKey(eKeys);
	static cSoftHdMenu *pSoftHdMenu;
	static cSoftHdMenu *Menu() { return pSoftHdMenu; }
};
