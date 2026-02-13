const { test, expect } = require('@playwright/test');

test('gimbal controls visible on DMTask', async ({ page }) => {
  await page.goto('/DMTask');
  const controls = page.getByTestId('pw-gimbal-controls');
  await expect(controls).toBeVisible();
});

test('pitch buttons visible', async ({ page }) => {
  await page.goto('/DMTask');
  const pitchRow = page.getByTestId('pw-gimbal-pitch-btns');
  await expect(pitchRow).toBeVisible();
  const buttons = pitchRow.locator('button');
  await expect(buttons).toHaveCount(6);
});

test('yaw buttons visible', async ({ page }) => {
  await page.goto('/DMTask');
  const yawRow = page.getByTestId('pw-gimbal-yaw-btns');
  await expect(yawRow).toBeVisible();
  const buttons = yawRow.locator('button');
  await expect(buttons).toHaveCount(6);
});

test('buttons disabled initially', async ({ page }) => {
  await page.goto('/DMTask');
  const pitchRow = page.getByTestId('pw-gimbal-pitch-btns');
  await expect(pitchRow).toBeVisible();
  const buttons = pitchRow.locator('button');
  const count = await buttons.count();
  for (let i = 0; i < count; i++) {
    await expect(buttons.nth(i)).toBeDisabled();
  }
});
