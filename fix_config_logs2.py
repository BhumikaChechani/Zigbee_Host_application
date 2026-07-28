import re

def fix_aqara(content):
    content = content.replace('printf("❌ ERROR: ', 'printf("ERROR: ')
    content = content.replace('printf("✅ SUCCESS: ', 'printf("SUCCESS: ')
    
    # Spatial Learning
    content = re.sub(r'LOG_DEBUG\("\[OCC\] AI Spatial Learning triggered on 0x%04X - keep room EMPTY for ".*?shortAddr_\);',
                     r'printf("SUCCESS: Triggering AI Spatial Learning on Aqara Occupancy 0x%04X... (Keep room empty for 30s)\\n", shortAddr_);', content, flags=re.DOTALL)
                     
    # Sensitivity
    content = content.replace('LOG_DEBUG("Sensitivity: 1=low 2=medium 3=high\\n");', 'printf("ERROR: Invalid sensitivity level. (1=low, 2=medium, 3=high)\\n");')
    content = re.sub(r'LOG_DEBUG\("Configuring Aqara Occupancy Sensor 0x%04X sensitivity to level %d ".*?\);',
                     r'printf("SUCCESS: Aqara Occupancy 0x%04X sensitivity set to level %d (%s).\\n", shortAddr_, level_, labels[level_]);', content, flags=re.DOTALL)

    return content

with open("src/aqara_occupancy.c", "r") as f:
    aqara_content = f.read()

with open("src/aqara_occupancy.c", "w") as f:
    f.write(fix_aqara(aqara_content))
