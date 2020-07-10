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


//////////////////////////////////////////////////////////////////////////////
//	cPlayer
//////////////////////////////////////////////////////////////////////////////

/**
**	player for mediaplayer mode.
*/
class cSoftHdPlayer : public cPlayer, cThread
{
//friend class cSoftHdControl;
private:
	void PlayFile(const char *path);
	char *Path;
	int StopFile;
protected:
	virtual void Activate(bool On);
	virtual void Action(void);
public:
	cSoftHdPlayer(const char *);
	virtual ~ cSoftHdPlayer();
	int Jump;
	int Pause;
};

//////////////////////////////////////////////////////////////////////////////
//	cControl
//////////////////////////////////////////////////////////////////////////////

/**
**	control class for mediaplayer mode.
*/

class cSoftHdControl : public cControl
{
//friend class cSoftHdMenu;
friend class cSoftHdPlayer;
private:
	static cSoftHdControl *pControl;
	virtual eOSState ProcessKey(eKeys);	///< process input events
	cSoftHdPlayer *pPlayer;
	int Close;
public:
	cSoftHdControl(const char *);		///< control constructor
	virtual ~cSoftHdControl();		///< control destructor

	virtual void Hide(void);		///< hide control
	static cSoftHdControl *Control() { return pControl; }
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
	void FindFile(string, FILE *);
	string Path;
	static cSoftHdControl *Control;
public:
	cSoftHdMenu(const char *, int = 0, int = 0, int = 0, int = 0, int = 0);
	virtual ~ cSoftHdMenu();
	virtual eOSState ProcessKey(eKeys);
};

