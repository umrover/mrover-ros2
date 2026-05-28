<template>
  <Teleport to="body">
    <div v-if="isOpen" class="modal-backdrop" @click.self="close">
      <div class="modal-dialog modal-wide">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">Import Waypoints</h5>
            <button type="button" class="btn-close" @click="close"><i class="bi bi-x-lg"></i></button>
          </div>
          <div class="modal-body">
            <div v-if="rows.length === 0" class="import-dropzone" @click="fileInput?.click()" @dragover.prevent @drop.prevent="onDrop">
              <i class="bi bi-file-earmark-arrow-up import-icon"></i>
              <p class="import-hint">Click to select or drag a file here</p>
              <p class="import-formats">.xlsx &nbsp;·&nbsp; .csv &nbsp;·&nbsp; .json</p>
            </div>

            <div v-else>
              <div class="import-summary mb-3">
                <span class="badge-count badge-update">{{ updateCount }} to update</span>
                <span class="badge-count badge-skip">{{ skipCount }} skipped (no coords)</span>
                <span v-if="notFoundCount > 0" class="badge-count badge-missing">{{ notFoundCount }} not in store</span>
              </div>
              <div class="preview-table-wrap">
                <table class="preview-table">
                  <thead>
                    <tr>
                      <th>Name</th>
                      <th>Lat</th>
                      <th>Lon</th>
                      <th>Status</th>
                    </tr>
                  </thead>
                  <tbody>
                    <tr v-for="(row, i) in rows" :key="i" :class="rowClass(row)">
                      <td>{{ row.name }}</td>
                      <td>{{ row.lat != null ? row.lat.toFixed(6) : '—' }}</td>
                      <td>{{ row.lon != null ? row.lon.toFixed(6) : '—' }}</td>
                      <td class="status-cell">{{ rowStatus(row) }}</td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </div>

            <p v-if="parseError" class="error-msg mt-2">{{ parseError }}</p>
          </div>
          <div class="modal-footer">
            <button v-if="rows.length > 0" class="btn btn-secondary btn-sm" @click="reset">Choose different file</button>
            <button class="btn btn-secondary btn-sm" @click="close">Cancel</button>
            <button
              v-if="updateCount > 0"
              class="btn btn-success btn-sm"
              :disabled="importing"
              @click="runImport"
            >
              {{ importing ? 'Importing…' : `Update ${updateCount} waypoint${updateCount !== 1 ? 's' : ''}` }}
            </button>
          </div>
        </div>
      </div>
    </div>
  </Teleport>

  <input ref="fileInput" type="file" accept=".xlsx,.csv,.json" class="hidden" @change="onFileChange" />
</template>

<script lang="ts" setup>
import { ref, computed } from 'vue'
import readXlsxFile from 'read-excel-file/browser'
import { useModal } from '@/composables/useModal'
import { useAutonomyStore } from '@/stores/autonomy'

interface ParsedRow {
  name: string
  lat: number | null
  lon: number | null
  dbId: number | null
}

const autonomyStore = useAutonomyStore()
const { isOpen, show, hide } = useModal()

const fileInput = ref<HTMLInputElement | null>(null)
const rows = ref<ParsedRow[]>([])
const parseError = ref('')
const importing = ref(false)

const updateCount = computed(() => rows.value.filter(r => r.dbId != null && r.lat != null && r.lon != null).length)
const skipCount = computed(() => rows.value.filter(r => r.dbId != null && (r.lat == null || r.lon == null)).length)
const notFoundCount = computed(() => rows.value.filter(r => r.dbId == null).length)

function open() { show() }

function close() {
  hide()
  reset()
}

function reset() {
  rows.value = []
  parseError.value = ''
  importing.value = false
  if (fileInput.value) fileInput.value.value = ''
}

function resolveDbId(name: string): number | null {
  const match = autonomyStore.store.find(wp => wp.name === name)
  return match?.db_id ?? null
}

function buildRows(records: { name: string; lat: number | null; lon: number | null }[]): void {
  rows.value = records.map(r => ({
    name: r.name,
    lat: r.lat,
    lon: r.lon,
    dbId: resolveDbId(r.name),
  }))
}

async function parseXlsx(file: File): Promise<void> {
  const sheets = await readXlsxFile(file)
  const firstSheet = sheets[0]
  if (!firstSheet || firstSheet.data.length < 2) {
    parseError.value = 'File has no data rows.'
    return
  }
  const rawRows = firstSheet.data
  const header = rawRows[0].map(c => String(c ?? '').trim().toLowerCase())
  const nameIdx = header.indexOf('name')
  const latIdx = header.indexOf('lat')
  const lonIdx = header.indexOf('lon')
  if (nameIdx === -1 || latIdx === -1 || lonIdx === -1) {
    parseError.value = 'File must have columns: name, lat, lon'
    return
  }
  const records = rawRows.slice(1).map(row => ({
    name: String(row[nameIdx] ?? '').trim(),
    lat: row[latIdx] != null && row[latIdx] !== '' ? Number(row[latIdx]) : null,
    lon: row[lonIdx] != null && row[lonIdx] !== '' ? Number(row[lonIdx]) : null,
  })).filter(r => r.name !== '')
  buildRows(records)
}

function parseCsv(text: string): void {
  const lines = text.split(/\r?\n/).filter(l => l.trim() !== '')
  if (lines.length < 2) {
    parseError.value = 'File has no data rows.'
    return
  }
  const header = lines[0].split(',').map(h => h.trim().toLowerCase())
  const nameIdx = header.indexOf('name')
  const latIdx = header.indexOf('lat')
  const lonIdx = header.indexOf('lon')
  if (nameIdx === -1 || latIdx === -1 || lonIdx === -1) {
    parseError.value = 'File must have columns: name, lat, lon'
    return
  }
  const records = lines.slice(1).map(line => {
    const cols = line.split(',')
    const latRaw = cols[latIdx]?.trim()
    const lonRaw = cols[lonIdx]?.trim()
    return {
      name: cols[nameIdx]?.trim() ?? '',
      lat: latRaw !== '' && latRaw != null ? Number(latRaw) : null,
      lon: lonRaw !== '' && lonRaw != null ? Number(lonRaw) : null,
    }
  }).filter(r => r.name !== '')
  buildRows(records)
}

function parseJson(text: string): void {
  let parsed: unknown
  try {
    parsed = JSON.parse(text)
  } catch {
    parseError.value = 'Invalid JSON.'
    return
  }
  if (!Array.isArray(parsed)) {
    parseError.value = 'JSON must be an array of objects.'
    return
  }
  const records = (parsed as Record<string, unknown>[]).map(obj => ({
    name: String(obj['name'] ?? '').trim(),
    lat: obj['lat'] != null && obj['lat'] !== '' ? Number(obj['lat']) : null,
    lon: obj['lon'] != null && obj['lon'] !== '' ? Number(obj['lon']) : null,
  })).filter(r => r.name !== '')
  buildRows(records)
}

async function processFile(file: File): Promise<void> {
  parseError.value = ''
  rows.value = []
  const ext = file.name.split('.').pop()?.toLowerCase()
  if (ext === 'xlsx') {
    await parseXlsx(file)
  } else if (ext === 'csv') {
    const text = await file.text()
    parseCsv(text)
  } else if (ext === 'json') {
    const text = await file.text()
    parseJson(text)
  } else {
    parseError.value = 'Unsupported file type. Use .xlsx, .csv, or .json.'
  }
}

function onFileChange(e: Event) {
  const file = (e.target as HTMLInputElement).files?.[0]
  if (file) processFile(file)
}

function onDrop(e: DragEvent) {
  const file = e.dataTransfer?.files?.[0]
  if (file) processFile(file)
}

async function runImport() {
  importing.value = true
  const toUpdate = rows.value.filter(r => r.dbId != null && r.lat != null && r.lon != null)
  await Promise.all(
    toUpdate.map(r => autonomyStore.updateStore(r.dbId!, { lat: r.lat!, lon: r.lon! }))
  )
  await autonomyStore.fetchAll()
  importing.value = false
  close()
}

function rowClass(row: ParsedRow): string {
  if (row.dbId != null && row.lat != null && row.lon != null) return 'row-update'
  if (row.dbId != null) return 'row-skip'
  return 'row-missing'
}

function rowStatus(row: ParsedRow): string {
  if (row.dbId != null && row.lat != null && row.lon != null) return 'Will update'
  if (row.dbId != null) return 'No coordinates'
  return 'Not in store'
}

defineExpose({ open, close })
</script>

<style scoped>
.modal-wide {
  max-width: 560px;
  width: 100%;
}

.import-dropzone {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
  padding: 2.5rem 1rem;
  border: 2px dashed var(--panel-border);
  border-radius: var(--radius-md);
  cursor: pointer;
  transition: border-color 0.15s;
}

.import-dropzone:hover {
  border-color: var(--text-muted);
}

.import-icon {
  font-size: 2rem;
  color: var(--text-muted);
}

.import-hint {
  font-size: 0.875rem;
  color: var(--text-muted);
  margin: 0;
}

.import-formats {
  font-size: 0.75rem;
  color: var(--text-muted);
  margin: 0;
}

.import-summary {
  display: flex;
  gap: 0.5rem;
  flex-wrap: wrap;
}

.badge-count {
  font-size: 0.75rem;
  font-weight: 600;
  padding: 0.2rem 0.5rem;
  border-radius: var(--radius-sm);
}

.badge-update {
  background-color: rgba(var(--status-ok-rgb), 0.2);
  color: var(--status-ok);
}

.badge-skip {
  background-color: rgba(var(--status-warn-rgb, 200, 160, 0), 0.2);
  color: var(--status-warn, #c8a000);
}

.badge-missing {
  background-color: rgba(var(--status-error-rgb, 180, 40, 40), 0.2);
  color: var(--status-error, #b42828);
}

.preview-table-wrap {
  max-height: 280px;
  overflow-y: auto;
  border: 1px solid var(--panel-border);
  border-radius: var(--radius-sm);
}

.preview-table {
  width: 100%;
  border-collapse: collapse;
  font-size: 0.8rem;
}

.preview-table th {
  padding: 0.4rem 0.6rem;
  text-align: left;
  font-weight: 600;
  background-color: var(--panel-bg);
  border-bottom: 1px solid var(--panel-border);
  position: sticky;
  top: 0;
}

.preview-table td {
  padding: 0.35rem 0.6rem;
  border-bottom: 1px solid var(--panel-border);
}

.preview-table tr:last-child td {
  border-bottom: none;
}

.row-update td { color: var(--text-primary); }
.row-skip td { color: var(--text-muted); }
.row-missing td { color: var(--status-error, #b42828); }

.status-cell {
  font-size: 0.7rem;
  font-style: italic;
}

.error-msg {
  font-size: 0.8rem;
  color: var(--status-error, #b42828);
}

.hidden {
  display: none;
}
</style>
