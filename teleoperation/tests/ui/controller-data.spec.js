const { test, expect } = require('@playwright/test');

test('data table visible on DMTask', async ({ page }) => {
  await page.goto('/DMTask');
  const statusToggle = page.getByTestId('pw-controller-status-toggle').first();
  await expect(statusToggle).toBeVisible();
});

test('status toggle clickable', async ({ page }) => {
  await page.goto('/DMTask');
  const statusToggle = page.getByTestId('pw-controller-status-toggle').first();
  await expect(statusToggle).toBeVisible();

  const initialClass = await statusToggle.getAttribute('class');
  await statusToggle.click();
  await page.waitForTimeout(300);
  const newClass = await statusToggle.getAttribute('class');
  expect(initialClass).not.toBe(newClass);
});

test('values toggle clickable', async ({ page }) => {
  await page.goto('/DMTask');
  const valuesToggle = page.getByTestId('pw-controller-values-toggle').first();
  await expect(valuesToggle).toBeVisible();

  const initialClass = await valuesToggle.getAttribute('class');
  await valuesToggle.click();
  await page.waitForTimeout(300);
  const newClass = await valuesToggle.getAttribute('class');
  expect(initialClass).not.toBe(newClass);
});
