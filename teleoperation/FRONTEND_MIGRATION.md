# Frontend Migration Guide: WebSocket to REST API

## Completed Migrations

### ✅ Backend
- Deprecated `waypoints_consumer.py` removed
- All REST API endpoints implemented and tested
- API utility created at `frontend/src/utils/api.ts`

### ✅ Frontend - All Components Migrated!
- `BasicWaypointEditor.vue` - Waypoint CRUD operations
- `AutonWaypointEditor.vue` - Autonomous navigation waypoints
- `WhiteLEDs.vue` - White LED control
- `LSActuator.vue` - Limit switch toggle
- `AutoShutdown.vue` - Auto shutoff control
- `NinhydrinBenedict.vue` - Heater control
- `SATask.vue` - Gear differential positioning
- `PanoCam.vue` - Panorama start/stop

## Migration Details

###  AutonWaypointEditor.vue

**File:** `frontend/src/components/AutonWaypointEditor.vue`

**WebSocket calls to replace:**

1. **Line ~264-269**: `save_auton_waypoint_list`
   ```javascript
   // OLD:
   this.$store.dispatch('websocket/sendMessage', {
     id: 'waypoints',
     message: { type: 'save_auton_waypoint_list', data: this.savedWaypoints }
   })

   // NEW:
   import { waypointsAPI } from '../api'
   await waypointsAPI.saveAuton(this.savedWaypoints)
   ```

2. **Line ~284-286**: `save_current_auton_course`
   ```javascript
   // OLD:
   this.$store.dispatch('websocket/sendMessage', {
     id: 'waypoints',
     message: { type: 'save_current_auton_course', data: this.currentWaypoints }
   })

   // NEW:
   await waypointsAPI.saveCurrentAutonCourse(this.currentWaypoints)
   ```

3. **Line ~346-351**: `get_auton_waypoint_list`
   ```javascript
   // OLD:
   this.$store.dispatch('websocket/sendMessage', {
     id: 'waypoints',
     message: { type: 'get_auton_waypoint_list' }
   })

   // NEW:
   const data = await waypointsAPI.getAuton()
   if (data.status === 'success') {
     this.savedWaypoints = data.waypoints
   }
   ```

4. **Line ~373-379**: `auton_enable` (start)
   ```javascript
   // OLD:
   this.$store.dispatch('websocket/sendMessage', {
     id: 'auton',
     message: { type: 'auton_enable', enabled: true, waypoints: formattedWaypoints }
   })

   // NEW:
   import { autonAPI } from '../api'
   await autonAPI.enable(true, formattedWaypoints)
   ```

5. **Line ~394-399**: `auton_enable` (stop)
   ```javascript
   // OLD:
   this.$store.dispatch('websocket/sendMessage', {
     id: 'auton',
     message: { type: 'auton_enable', enabled: false, waypoints: [] }
   })

   // NEW:
   await autonAPI.enable(false, [])
   ```

6. **Line ~410-415**: `delete_auton_waypoint_from_course`
   ```javascript
   // OLD:
   this.$store.dispatch('websocket/sendMessage', {
     id: 'waypoints',
     message: { type: 'delete_auton_waypoint_from_course', data: waypoint }
   })

   // NEW:
   await waypointsAPI.deleteAutonWaypoint(waypoint)
   ```

7. **Line ~477-481**: `teleop_enable`
   ```javascript
   // OLD:
   this.$store.dispatch('websocket/sendMessage', {
     id: 'auton',
     message: { type: 'teleop_enable', enabled }
   })

   // NEW:
   await autonAPI.enableTeleop(enabled)
   ```

**Additional changes:**
- Remove `...mapActions('websocket', ['sendMessage'])` from methods
- Remove WebSocket message watchers for waypoint/auton responses
- Import API utilities: `import { waypointsAPI, autonAPI } from '../api'`

---

### Science Components

#### SelectSite.vue / NinhydrinBenedict.vue
**WebSocket call:** `set_gear_diff_pos`

```javascript
// OLD:
this.$store.dispatch('websocket/sendMessage', {
  id: 'science',
  message: { type: 'set_gear_diff_pos', position, isCCW }
})

// NEW:
import { scienceAPI } from '../api'
await scienceAPI.setGearDiffPosition(position, isCCW)
```

#### WhiteLEDs.vue
**WebSocket call:** `white_leds`

```javascript
// OLD:
this.$store.dispatch('websocket/sendMessage', {
  id: 'science',
  message: { type: 'white_leds', site, enable }
})

// NEW:
await scienceAPI.setWhiteLEDs(site, enable)
```

#### LSActuator.vue
**WebSocket call:** `ls_toggle`

```javascript
// OLD:
this.$store.dispatch('websocket/sendMessage', {
  id: 'science',
  message: { type: 'ls_toggle', enable }
})

// NEW:
await scienceAPI.setLimitSwitch(enable)
```

#### AutoShutdown.vue
**WebSocket call:** `auto_shutoff`

```javascript
// OLD:
this.$store.dispatch('websocket/sendMessage', {
  id: 'science',
  message: { type: 'auto_shutoff', shutoff }
})

// NEW:
await scienceAPI.setAutoShutoff(shutoff)
```

#### SATask.vue (or wherever heaters are controlled)
**WebSocket call:** `heater_enable`

```javascript
// OLD:
this.$store.dispatch('websocket/sendMessage', {
  id: 'science',
  message: { type: 'heater_enable', enable, heater: 'a0' }
})

// NEW:
await scienceAPI.setHeater('a0', enable)
```

---

### Mast Component

#### PanoCam.vue
**WebSocket call:** `pano`

```javascript
// OLD (start):
this.$store.dispatch('websocket/sendMessage', {
  id: 'mast',
  message: { type: 'pano', action: 'start' }
})

// NEW:
import { mastAPI } from '../api'
await mastAPI.startPanorama()

// OLD (stop):
this.$store.dispatch('websocket/sendMessage', {
  id: 'mast',
  message: { type: 'pano', action: 'stop' }
})

// NEW:
const result = await mastAPI.stopPanorama()
if (result.status === 'success') {
  console.log('Panorama saved to:', result.image_path)
}
```

---

## Migration Checklist

For each component:
- [ ] Import API utilities from `../api`
- [ ] Replace `this.$store.dispatch('websocket/sendMessage', ...)` with API calls
- [ ] Remove unused WebSocket imports/mappings
- [ ] Remove WebSocket response watchers (if present)
- [ ] Add error handling for API calls
- [ ] Test the component functionality

## Testing

After migration:
1. Start Django backend: `python manage.py runserver`
2. Ensure djangorestframework is installed: `pip install djangorestframework`
3. Test each migrated component in the GUI
4. Check browser DevTools Network tab to verify REST calls
5. Check for any console errors

## Rollback Plan

If issues arise:
1. Git checkout original files
2. Restore `waypoints_consumer.py` if needed
3. Report issues in API.md or this file
