from krpc.services.spacecenter import Vessel
from krpc.client import Client

def get_vessel_by_name_and_tag(conn: Client, name: str, tag:str, is_part_of:bool=False)-> Vessel:
    """
    Return the vessel with same name, and has one part with the same tag

    Args:
        name:
        tag:
        is_part_of: If True will look for a vessel that has input name as part of entire name
    
    Returns:
        Vessel
    """
    vessel_list = conn.space_center.vessels
    for vessel in vessel_list:
        if (vessel.name.strip() == name.strip() and is_part_of==False) or ( name.strip() in vessel.name.strip() and is_part_of==True):
            for part in vessel.parts.all:
                if part.tag.strip() == tag.strip():
                    return vessel
                
    return None