const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('gimbal controls visible on DMTask', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const controls = page.getByTestId('pw-gimbal-controls');
  await expect(controls).toBeVisible({ timeout: 15000 });
});

test('pitch buttons visible', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const pitchRow = page.getByTestId('pw-gimbal-pitch-btns');
  await expect(pitchRow).toBeVisible({ timeout: 15000 });
  const buttons = pitchRow.locator('button');
  await expect(buttons).toHaveCount(6);
});

test('yaw buttons visible', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const yawRow = page.getByTestId('pw-gimbal-yaw-btns');
  await expect(yawRow).toBeVisible({ timeout: 15000 });
  const buttons = yawRow.locator('button');
  await expect(buttons).toHaveCount(6);
});

test('buttons disabled initially', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const pitchRow = page.getByTestId('pw-gimbal-pitch-btns');
  await expect(pitchRow).toBeVisible({ timeout: 15000 });
  const buttons = pitchRow.locator('button');
  const count = await buttons.count();
  for (let i = 0; i < count; i++) {
    await expect(buttons.nth(i)).toBeDisabled();
  }
});
