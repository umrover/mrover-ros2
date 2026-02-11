const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('data table visible on DMTask', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const statusToggle = page.getByTestId('pw-controller-status-toggle').first();
  await expect(statusToggle).toBeVisible({ timeout: 15000 });
});

test('status toggle clickable', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const statusToggle = page.getByTestId('pw-controller-status-toggle').first();
  await expect(statusToggle).toBeVisible({ timeout: 15000 });

  const initialClass = await statusToggle.getAttribute('class');
  await statusToggle.click();
  await page.waitForTimeout(300);
  const newClass = await statusToggle.getAttribute('class');
  expect(initialClass).not.toBe(newClass);
});

test('values toggle clickable', async ({ page }) => {
  await page.goto('http://localhost:8080/DMTask');
  await page.waitForLoadState('networkidle');
  const valuesToggle = page.getByTestId('pw-controller-values-toggle').first();
  await expect(valuesToggle).toBeVisible({ timeout: 15000 });

  const initialClass = await valuesToggle.getAttribute('class');
  await valuesToggle.click();
  await page.waitForTimeout(300);
  const newClass = await valuesToggle.getAttribute('class');
  expect(initialClass).not.toBe(newClass);
});
