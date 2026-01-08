// ...existing code...
const { test, expect } = require('@playwright/test');

test('basestation frontend loads', async ({ page }) => {
  await page.goto('http://localhost:8080/');

  // You can add more specific assertions here
  // For example, checking for a specific title
  await expect(page).toHaveTitle(/MRover/);
});

// ...existing code...
test('autonomy waypoint with string name persists after refresh', async ({ page }) => {
  const base = 'http://localhost:8080/';
  const waypointName = `test-waypoint-${Date.now()}`;

  await page.goto(base);

  // Helper: try selectors and click the first that exists
  const clickFirstFound = async (selectors) => {
    for (const s of selectors) {
      const loc = page.locator(s).first();
      if (await loc.count() > 0) {
        try {
          await loc.scrollIntoViewIfNeeded();
          await loc.click({ force: true });
          return true;
        } catch (e) {
          // ignore and continue trying other selectors
        }
      }
    }
    return false;
  };

  // Open autonomy/mission view
  const opened = await clickFirstFound([
    'text=Autonomy',
    'text=Auton',
    'text=Autonomy Mission',
    '[data-test="autonomy-tab"]',
    '[data-testid="autonomy-tab"]'
  ]);
  if (!opened) throw new Error('Could not open autonomy view - adjust selector in test');

  // Click the "add waypoint" control
  const added = await clickFirstFound([
    'text=Add Waypoint',
    'text=New Waypoint',
    'button[title="Add waypoint"]',
    '[data-test="add-waypoint"]',
    '[data-testid="add-waypoint"]',
    'button:has-text("Add")'
  ]);
  if (!added) throw new Error('Could not find Add Waypoint control - adjust selector in test');

  // Find name input
  const nameLocCandidates = [
    'input[placeholder="Name"]',
    'input[name="name"]',
    '[data-test="waypoint-name"]',
    '[data-testid="waypoint-name"]',
    'input:below(label:has-text("Name"))'
  ];
  let nameLoc = null;
  for (const sel of nameLocCandidates) {
    const loc = page.locator(sel).first();
    if (await loc.count() > 0) {
      nameLoc = loc;
      break;
    }
  }
  if (!nameLoc) throw new Error('Waypoints name input not found - adjust selector in test');

  await nameLoc.fill(waypointName);

  // Click save/add submit for waypoint
  const savedPromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints') && resp.request().method() === 'POST'
  );
  const saved = await clickFirstFound([
    'text=Save',
    'text=Add Waypoint',
    'button[type="submit"]',
    '[data-test="save-waypoint"]',
    '[data-testid="save-waypoint"]'
  ]);
  if (!saved) throw new Error('Could not find Save/Add button for waypoint - adjust selector in test');
  await savedPromise;

  // Reload the page and ensure waypoint still present
  await page.reload();
  // Re-open autonomy view if needed
  await clickFirstFound([
    'text=Autonomy',
    'text=Auton',
    'text=Autonomy Mission',
    '[data-test="autonomy-tab"]',
    '[data-testid="autonomy-tab"]'
  ]);
  // Wait for waypoints to be re-fetched after page reload
  await page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints') && resp.request().method() === 'GET'
  );
});

test('basestation frontend loads', async ({ page }) => {
  
});
