import re

with open("src/siren.c", "r") as f:
    content = f.read()

# Remove global volume and mode variables
content = re.sub(r'static uint8_t s_sirenVolume = 2; // high by default\n?', '', content)
content = re.sub(r'static uint8_t s_sirenMode = 1;   // burglar by default\n?', '', content)

# Remove LoadConfig and SaveConfig
content = re.sub(r'static const char SIREN_CONFIG_FILE\[\] = "siren_config\.txt";.*?static void Siren_SaveConfig\(void\).*?}', '', content, flags=re.DOTALL)

# Remove LoadConfig from Init
content = content.replace('Siren_LoadConfig();', '')

# Initialize volume and mode in Siren_Discover
content = content.replace('g_sirens[g_numSirens].configured = false;',
                          'g_sirens[g_numSirens].configured = false;\n            g_sirens[g_numSirens].volume = 2;\n            g_sirens[g_numSirens].mode = 1;')

# Replace SetVolume
content = re.sub(r'void Siren_SetVolume\( uint8_t volume_ \).*?}',
                 r'''void Siren_SetVolume( uint16_t shortAddr_, uint8_t volume_ )
{
    if (volume_ > 3) volume_ = 3;
    pthread_mutex_lock(&g_deviceMutex);
    bool found = false;
    for (int i = 0; i < g_numSirens; i++) {
        if (g_sirens[i].shortAddr == shortAddr_) {
            g_sirens[i].volume = volume_;
            found = true;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    if (found) {
        Device_Save();
        printf("SUCCESS: Siren 0x%04X volume set to %d (0=low, 1=medium, 2=high, 3=very high).\\n", shortAddr_, volume_);
    } else {
        printf("ERROR: Siren 0x%04X not found.\\n", shortAddr_);
    }
}''', content, flags=re.DOTALL)

# Replace SetMode
content = re.sub(r'void Siren_SetMode\( uint8_t mode_ \).*?}',
                 r'''void Siren_SetMode( uint16_t shortAddr_, uint8_t mode_ )
{
    if (mode_ < 1 || mode_ > 6) mode_ = 1;
    pthread_mutex_lock(&g_deviceMutex);
    bool found = false;
    for (int i = 0; i < g_numSirens; i++) {
        if (g_sirens[i].shortAddr == shortAddr_) {
            g_sirens[i].mode = mode_;
            found = true;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    if (found) {
        Device_Save();
        printf("SUCCESS: Siren 0x%04X mode set to %d.\\n", shortAddr_, mode_);
    } else {
        printf("ERROR: Siren 0x%04X not found.\\n", shortAddr_);
    }
}''', content, flags=re.DOTALL)

# Update Siren_ControlAllDuration
content = content.replace('uint8_t mode = (warnMode_ != 0) ? s_sirenMode : 0;\n        ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), mode, s_sirenVolume, durationSeconds_ );',
                          'uint8_t mode = (warnMode_ != 0) ? tempSirens[i].mode : 0;\n        ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), mode, tempSirens[i].volume, durationSeconds_ );')

# Update Siren_ControlSquawk
content = content.replace('ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), s_sirenMode, squawkLevel_, 1 );',
                          'ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), tempSirens[i].mode, squawkLevel_, 1 );')

# Add Siren_Control
content = content.replace('void Siren_ControlAllDuration( uint8_t warnMode_, uint16_t durationSeconds_ )',
                          '''void Siren_Control( uint16_t shortAddr_, uint8_t warnMode_ )
{
    g_sirenActive = ( warnMode_ != 0 );
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if (g_sirens[i].shortAddr == shortAddr_) {
            uint8_t mode = (warnMode_ != 0) ? g_sirens[i].mode : 0;
            uint8_t ep = g_sirens[i].endpoint;
            uint8_t vol = g_sirens[i].volume;
            pthread_mutex_unlock( &g_deviceMutex );
            ZNP_SendSirenWarning( shortAddr_, ep, Siren_NextSeq(), mode, vol, 240 );
            return;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

void Siren_ControlAllDuration( uint8_t warnMode_, uint16_t durationSeconds_ )''')

with open("src/siren.c", "w") as f:
    f.write(content)
