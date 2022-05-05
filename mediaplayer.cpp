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

#include <cstdlib>
#include <sys/stat.h>

#include <string>
using std::string;
#include <fstream>
using std::ifstream;
#include <sys/stat.h>

#include <vdr/interface.h>
#include <vdr/player.h>
#include <vdr/plugin.h>
#include <vdr/videodir.h>

#include "mediaplayer.h"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>

#include "softhddev.h"
#include "audio.h"
}

// fix temporary array error in c++1x
#ifdef av_err2str
#undef av_err2str
av_always_inline char* av_err2str(int errnum)
{
	static char str[AV_ERROR_MAX_STRING_SIZE];
	memset(str, 0, sizeof(str));
	return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}
#endif

//////////////////////////////////////////////////////////////////////////////
//	cPlayer Mediaplayer
//////////////////////////////////////////////////////////////////////////////

cSoftHdPlayer::cSoftHdPlayer(const char *Url)
:cPlayer (pmAudioVideo)
{
//	pPlayer= this;
	Source = (char *) malloc(1 + strlen(Url));
	strcpy(Source, Url);
	if (strcasestr(Source, ".M3U") && !strcasestr(Source, ".M3U8")) {
		ReadPL(Source);
		CurrentEntry = FirstEntry;
	} else {
		FirstEntry = CurrentEntry = NULL;
	}
	Pause = 0;
	Random = 0;
//	fprintf(stderr, "cSoftHdPlayer: Player gestartet.\n");
}

cSoftHdPlayer::~cSoftHdPlayer()
{
	StopPlay = 1;
	free(Source);
	if (FirstEntry) {
		while(FirstEntry) {
			PLEntry *entry = FirstEntry;
			FirstEntry = entry->NextEntry;
			delete entry;
			Entries--;
		}
	}

//	fprintf(stderr, "cSoftHdPlayer: Player beendet.\n");
}

void cSoftHdPlayer::Activate(bool On)
{
//	fprintf(stderr, "cSoftHdPlayer: Activate %s\n", On ? "On" : "Off");
	if (On)
		Start();
}

void cSoftHdPlayer::Action(void)
{
//	fprintf(stderr, "cSoftHdPlayer: Action\n");
	NoModify = 0;

	if (strcasestr(Source, ".M3U") && !strcasestr(Source, ".M3U8")) {
		while(CurrentEntry) {
			Jump = 0;
			Player(CurrentEntry->Path.c_str());

			if (!NoModify) {
				CurrentEntry = CurrentEntry->NextEntry;

				if (Random) {
					srand (time (NULL));
					SetEntry(std::rand() % (Entries));
				}
			}
			NoModify = 0;

			if (cSoftHdMenu::Menu()) {
				cSoftHdMenu::Menu()->PlayListMenu();
			}
		}
	} else {
		Player(Source);
	}

	while(AudioGetClock() != AV_NOPTS_VALUE)
		usleep(5000);

	cSoftHdControl::Control()->Close = 1;
}

void cSoftHdPlayer::ReadPL(const char *Playlist)
{
	ifstream f;
	PLEntry *last_entry = NULL;
	Entries = 0;

	f.open(Playlist);
	if (!f.good()) {
		fprintf(stderr, "Mediaplayer: open PL %s failed\n", Playlist);
		return;
	}

	while (!f.eof()) {
		string s;
		getline(f, s);
		if (s.size() && s.compare(0, 1, "#")) {
			PLEntry *entry = new PLEntry;
			entry->NextEntry = NULL;

			entry->Path = s;
			entry->File = entry->Path.substr(entry->Path.find_last_of("/") +1, string::npos);

			string SubString = entry->Path.substr(0, entry->Path.find_last_of("/"));
			entry->SubFolder = SubString.substr(SubString.find_last_of("/") +1, string::npos);

			string FolderString = entry->Path.substr(0, SubString.find_last_of("/"));
			entry->Folder = FolderString.substr(FolderString.find_last_of("/") +1, string::npos);

			if (!last_entry) {
				FirstEntry = entry;
			} else {
				last_entry->NextEntry = entry;
			}
			last_entry = entry;
			Entries++;
		}
	}

	f.close();
}

void cSoftHdPlayer::SetEntry(int index)
{
	PLEntry *entry;
	entry = FirstEntry;

	for(int i = 0; i < index ; i++) {
		entry = entry->NextEntry;
	}
	CurrentEntry = entry;
	NoModify = 1;
	StopPlay = 1;
}

void cSoftHdPlayer::Player(const char *url)
{
	AVPacket packet;
	AVCodec *video_codec;
	int err = 0;
	int audio_stream_index = 0;
	int video_stream_index;
	int jump_stream_index = 0;
	int start_time;

	StopPlay = 0;
	Jump = 0;

	AVFormatContext *format = avformat_alloc_context();
	if (avformat_open_input(&format, url, NULL, NULL) != 0) {
		fprintf(stderr, "Mediaplayer: Could not open file '%s'\n", url);
		return;
	}
#ifdef MEDIA_DEBUG
	av_dump_format(format, -1, url, 0);
#endif
	if (avformat_find_stream_info(format, NULL) < 0) {
		fprintf(stderr, "Mediaplayer: Could not retrieve stream info from file '%s'\n", url);
		return;
	}

	for (unsigned int i = 0; i < format->nb_streams; i++) {
		if (format->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
			SetAudioCodec(format->streams[i]->codecpar->codec_id,
				format->streams[i]->codecpar, &format->streams[i]->time_base);
			audio_stream_index = jump_stream_index = i;
			break;
		}
	}

	video_stream_index = av_find_best_stream(format, AVMEDIA_TYPE_VIDEO,
		-1, -1, &video_codec, 0);

	if (video_stream_index < 0) {
#ifdef MEDIA_DEBUG
		fprintf(stderr, "Player: stream does not seem to contain video\n");
#endif
	} else {
		 SetVideoCodec(video_codec->id,
			format->streams[video_stream_index]->codecpar,
			&format->streams[video_stream_index]->time_base);
		jump_stream_index = video_stream_index;
	}

	Duration = format->duration / AV_TIME_BASE;
	start_time = format->start_time / AV_TIME_BASE;

	while (!StopPlay) {
		err = av_read_frame(format, &packet);
		if (err == 0) {
repeat:
			if (audio_stream_index == packet.stream_index) {
				if (!PlayAudioPkts(&packet)) {
					usleep(packet.duration * AV_TIME_BASE *
						av_q2d(format->streams[audio_stream_index]->time_base));
					goto repeat;
				}
				CurrentTime = AudioGetClock() / 1000 - start_time;
			}

			if (video_stream_index == packet.stream_index) {
				if (!PlayVideoPkts(&packet)) {
					usleep(packet.duration * AV_TIME_BASE *
						av_q2d(format->streams[video_stream_index]->time_base));
					goto repeat;
				}
			}
		} else {
#ifdef MEDIA_DEBUG
			fprintf(stderr, "Player: av_read_frame error: %s\n",
				av_err2str(err));
#endif
			StopPlay = 1;
			continue;
		}

		while (Pause) {
			sleep(1);
		}

		if (Jump && format->pb->seekable) {
			av_seek_frame(format, format->streams[jump_stream_index]->index,
				packet.pts + (int64_t)(Jump /		// - BufferOffset
				av_q2d(format->streams[jump_stream_index]->time_base)), 0);
			Clear();
			Jump = 0;
		}

		if (StopPlay)
			Clear();

		av_packet_unref(&packet);
	}

	Duration = 0;
	CurrentTime = 0;

	avformat_close_input(&format);
	avformat_free_context(format);
}

const char * cSoftHdPlayer::GetTitle(void)
{
	if (CurrentEntry)
		return CurrentEntry->Path.c_str();

	return Source;
}

//////////////////////////////////////////////////////////////////////////////
//	cControl Mediaplayer
//////////////////////////////////////////////////////////////////////////////

cSoftHdControl *cSoftHdControl::pControl = NULL;
cSoftHdPlayer *cSoftHdControl::pPlayer = NULL;

/**
**	Player control constructor.
*/
cSoftHdControl::cSoftHdControl(const char *Url)
:cControl(pPlayer = new cSoftHdPlayer(Url))
{
//	fprintf(stderr, "cSoftHdControl: Player gestartet.\n");
	pControl = this;
	Close = 0;
	pOsd = NULL;
}

/**
**	Player control destructor.
*/
cSoftHdControl::~cSoftHdControl()
{
	delete pPlayer;
	pPlayer = NULL;
	pControl = NULL;
#ifdef MEDIA_DEBUG
	fprintf(stderr, "cSoftHdControl: Player beendet.\n");
#endif
}

void cSoftHdControl::Hide(void)
{
#ifdef MEDIA_DEBUG
	fprintf(stderr, "[cSoftHdControl] Hide\n");
#endif
	if (pOsd) {
		delete pOsd;
		pOsd = NULL;
	}
}

void cSoftHdControl::ShowProgress(void)
{
	if (!pOsd) {
#ifdef MEDIA_DEBUG
		fprintf(stderr, "[cSoftHdControl] ShowProgress get OSD\n");
#endif
		pOsd = Skins.Current()->DisplayReplay(false);
	}

	pOsd->SetTitle(pPlayer->GetTitle());
	pOsd->SetProgress(pPlayer->CurrentTime, pPlayer->Duration);
	pOsd->SetCurrent(IndexToHMSF(pPlayer->CurrentTime, false, 1));
	pOsd->SetTotal(IndexToHMSF(pPlayer->Duration, false, 1));

	Skins.Flush();
}

/**
**	Handle a key event.
**
**	@param key	key pressed
*/
eOSState cSoftHdControl::ProcessKey(eKeys key)
{
	switch (key) {
		case kNone:
			if (pOsd)
				ShowProgress();
			if (Close) {
				Hide();
				return osStopReplay;
			}
			break;

		case kOk:
			if (pOsd) {
				Hide();
			} else {
				ShowProgress();
			}
			break;

		case kPlay:
			if (pPlayer->Pause) {
				pPlayer->Pause = 0;
				Play();
			}
			break;

		case kGreen:
			pPlayer->Jump = -60;
		break;

		case kYellow:
			pPlayer->Jump = 60;
		break;

		case kBlue:
			Hide();
			pPlayer->StopPlay = 1;
			return osStopReplay;

		case kPause:
			if (pPlayer->Pause) {
				pPlayer->Pause = 0;
				Play();
			} else {
				pPlayer->Pause = 1;
				Freeze();
			}
			break;

		case kNext:
			pPlayer->StopPlay = 1;
			break;

		default:
			break;
	}

	return osContinue;
}


//////////////////////////////////////////////////////////////////////////////
//	cOsdMenu
//////////////////////////////////////////////////////////////////////////////
cSoftHdMenu *cSoftHdMenu::pSoftHdMenu = NULL;

/**
**	Soft device menu constructor.
*/
cSoftHdMenu::cSoftHdMenu(const char *title, int c0, int c1, int c2, int c3,
    int c4)
:cOsdMenu(title, c0, c1, c2, c3, c4)
{
	pSoftHdMenu = this;
	Playlist.clear();

	if (cSoftHdControl::Control() && cSoftHdControl::Control()->Player()->FirstEntry) {
#ifdef MEDIA_DEBUG
		fprintf(stderr, "cSoftHdMenu: pointer to cSoftHdControl exist.\n");
#endif
		PlayListMenu();		// Test if PL!!!
	} else {
		MainMenu();
	}
}

/**
**	Soft device menu destructor.
*/
cSoftHdMenu::~cSoftHdMenu()
{
	pSoftHdMenu = NULL;
}

/**
**	Create main menu.
*/
void cSoftHdMenu::MainMenu(void)
{
	int current;
	int duped;
	int dropped;
	int counter;

	current = Current();		// get current menu item index
	Clear();				// clear the menu

	Add(new cOsdItem(hk(tr(" play file / make play list")), osUser1));
	Add(new cOsdItem(hk(tr(" select play list")), osUser2));
	Add(new cOsdItem(NULL, osUnknown, false));
	Add(new cOsdItem(NULL, osUnknown, false));
	GetStats(&duped, &dropped, &counter);
	Add(new cOsdItem(cString::sprintf(tr
		(" Frames duped(%d) dropped(%d) total(%d)"),
		duped, dropped, counter), osUnknown, false));

	SetCurrent(Get(current));		// restore selected menu entry
	Display();
}

/**
**	Create play list menu.
*/
void cSoftHdMenu::PlayListMenu(void)
{
	struct PLEntry *entry = cSoftHdControl::Control()->Player()->FirstEntry;
	Clear();
	while (1) {
		string String = entry->Folder
			+ " - " + entry->SubFolder
			+ " - " + entry->File;
		Add(new cOsdItem(String.c_str()), (entry == cSoftHdControl::Control()->Player()->CurrentEntry));

		if (entry->NextEntry) {
			entry = entry->NextEntry;
		} else {
			break;
		}
	}
	SetHelp(cSoftHdControl::Control()->Player()->Random ? "Random Play" : " No Random Play",
		"Jump -1 min", "Jump +1 min", "End player");
	Display();
}

/**
**	Create select play list menu.
*/
void cSoftHdMenu::SelectPL(void)
{
	struct dirent **DirList;
	int n, i;

	if ((n = scandir(cPlugin::ConfigDirectory("softhddevice-drm"), &DirList, NULL, alphasort)) == -1) {
		fprintf(stderr, "SelectPL: searching PL in %s failed (%d): %m\n",
			cPlugin::ConfigDirectory("softhddevice-drm"), errno);
	} else {
		Clear();
		for (i = 0; i < n; i++) {
			if (DirList[i]->d_name[0] != '.' && (strcasestr(DirList[i]->d_name, ".M3U"))) {
				Add(new cOsdItem(DirList[i]->d_name));
			}
		}
		SetHelp("Play PL", NULL, NULL, NULL);
		Display();
	}
}

/**
**	Create sub menu find file or make a play list.
**
**	@param SearchPath	path to start search mediafile
**	@param playlist		if there is a play list write to play list else make a new menu
*/
void cSoftHdMenu::FindFile(string SearchPath, FILE *playlist)
{
	struct dirent **DirList;
	int n, i;
	const char * sp;

	if (!SearchPath.size())
		sp = "/";
	else sp = SearchPath.c_str();

	if (!playlist) {
		Clear();
		if (SearchPath.size())
			Add(new cOsdItem("[..]"));
	}

	if ((n = scandir(sp, &DirList, NULL, alphasort)) == -1) {
		fprintf(stderr, "FindFile: scanning directory %s failed (%d): %m\n", sp, errno);
	} else {
		struct stat fileAttributs;
		for (i = 0; i < n; i++) {
			string str = SearchPath + "/" + DirList[i]->d_name;
			if (stat(str.c_str(), &fileAttributs) == -1) {
				fprintf(stderr, "FindFile: stat on %s failed (%d): %m\n", str.c_str(), errno);
			} else {
			if (S_ISDIR(fileAttributs.st_mode) && DirList[i]->d_name[0] != '.') {
				if (playlist) {
					FindFile(str.c_str(), playlist);
				} else {
					Add(new cOsdItem(DirList[i]->d_name),
						!LastItem.compare(0, LastItem.length(), DirList[i]->d_name));
				}
			}
			}
		}
		for (i = 0; i < n; i++) {
			string str = SearchPath + "/" + DirList[i]->d_name;
			if (stat(str.c_str(), &fileAttributs) == -1) {
				fprintf(stderr, "FindFile: stat on %s failed (%d): %m\n", str.c_str(), errno);
			} else {
			if (S_ISREG(fileAttributs.st_mode) && DirList[i]->d_name[0] != '.') {
				if (playlist) {
					if (TestMedia(DirList[i]->d_name))
						fprintf(playlist, "%s/%s\n", SearchPath.c_str(),
							DirList[i]->d_name);
				} else {
					Add(new cOsdItem(DirList[i]->d_name));
				}
			}
			}
		}
	}

	if (!playlist) {
		SetHelp( Playlist.empty() ? "Play File" : "Play PL", "New PL", "Add to PL", NULL);
//		SetHelp(Control->Player->Running ? NULL : "Set new PL",
//			Control->Player->Running ? "Play Menu" : "Select PL");
		Display();
	}
}

/**
**	Make a play list.
**
**	@param Target	path to start search mediafiles
**	@param mode		open file mode
*/
void cSoftHdMenu::MakePlayList(const char * Target, const char * mode)
{
	if (Playlist.empty())
		Playlist = "/default.m3u";		// if (!Playlist) ???

	string PlPath = cPlugin::ConfigDirectory("softhddevice-drm");
	PlPath.append(Playlist.c_str());
	FILE *playlist = fopen(PlPath.c_str(), mode);

	if (playlist != NULL) {
		if (TestMedia(Target)) {
			fprintf(playlist, "%s/%s\n", Path.c_str(), Target);
		} else {
			string str = Path + "/" + Target;
			FindFile(str.c_str(), playlist);
		}
	fclose (playlist);
	}
}

/**
**	Handle key event.
**
**	@param key	key event
*/
eOSState cSoftHdMenu::ProcessKey(eKeys key)
{
	eOSState state;
	cOsdItem *item;

	item = (cOsdItem *) Get(Current());
	state = cOsdMenu::ProcessKey(key);

	switch (state) {
		case osUser1:			// play file / make play list
			Path = cVideoDirectory::Name();
			FindFile(Path, NULL);
			return osContinue;
		case osUser2:			// select play list
			Path = cPlugin::ConfigDirectory("softhddevice-drm");
			SelectPL();
			return osContinue;
		default:
			break;
	}

	switch (key) {
		case kOk:
			if (strcasestr(item->Text(), "[..]")) {
				string NewPath = Path.substr(0 ,Path.find_last_of("/"));

				if (!LastItem.empty())
					LastItem.clear();
				LastItem = Path.substr(Path.find_last_of("/") + 1);

				Path = NewPath;
				FindFile(Path.c_str(), NULL);
				break;
			}
			if (cSoftHdControl::Control() && cSoftHdControl::Control()->Player()->CurrentEntry) {
				cSoftHdControl::Control()->Player()->SetEntry(Current());
//				PlayListMenu();
				break;
			}
			if (TestMedia(item->Text())) {
				PlayMedia(item->Text());
				return osEnd;
			} else {
				string NewPath = Path + "/" + item->Text();
				struct stat sb;
				if (stat(NewPath.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
					Path = NewPath;
					FindFile(NewPath.c_str(), NULL);
				}
			}
			break;
		case kRed:
			if (cSoftHdControl::Control() && cSoftHdControl::Control()->Player()->CurrentEntry) {
				cSoftHdControl::Control()->Player()->Random ^= 1;
				PlayListMenu();
				break;
			}
			if (Playlist.empty()) {
				if (TestMedia(item->Text())) {
					PlayMedia(item->Text());
					return osEnd;
				}
			} else {
				Path = cPlugin::ConfigDirectory("softhddevice-drm");
				PlayMedia(Playlist.c_str());
				return osEnd;
			}
			break;
		case kGreen:
			if (cSoftHdControl::Control()) {
				cSoftHdControl::Control()->Player()->Jump = -60;
			} else {
				MakePlayList(item->Text(), "w");
				Interface->Confirm(tr("New Playlist"), 1, true);
				if (!LastItem.empty())
					LastItem.clear();
				LastItem = item->Text();
				FindFile(Path.c_str(), NULL);
			}
			break;
		case kYellow:
			if (cSoftHdControl::Control()) {
				cSoftHdControl::Control()->Player()->Jump = 60;
			} else {
				MakePlayList(item->Text(), "a");
				Interface->Confirm(tr("Added to Playlist"), 1, true);
			}
			break;
		case kBlue:
			state = osStopReplay;
			break;
		case kPlay:
			if (TestMedia(item->Text())) {
				PlayMedia(item->Text());
				return osEnd;
			}
			break;
		case kNext:
			if (cSoftHdControl::Control())
				cSoftHdControl::Control()->Player()->StopPlay = 1;
			break;
		default:
			break;
	}

	return state;
}

/**
**	Play media file.
**
**	@param name	file name
*/
void cSoftHdMenu::PlayMedia(const char *name)
{
	string aim = Path + "/" + name;
	if (!cSoftHdControl::Control()) {
		cControl::Launch(new cSoftHdControl(aim.c_str()));
	} else {
		fprintf(stderr, "PlayMedia can't start %s\n",aim.c_str());
	}
}

/**
**	Test if it's a media file.
**
**	@param name	file name
**	@returns true if it's a media file
*/
int cSoftHdMenu::TestMedia(const char *name)
{
	if (strcasestr(name, ".MP3"))
		return 1;
	if (strcasestr(name, ".MP4"))
		return 1;
	if (strcasestr(name, ".M3U"))
		return 1;
	if (strcasestr(name, ".TS"))
		return 1;

	return 0;
}
